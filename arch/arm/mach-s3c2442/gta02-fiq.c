#include <linux/kernel.h>

#include <asm/fiq.h>
#include <plat/pwm.h>
#include <mach/regs-irq.h>
#include <mach/irqs.h>
#include <linux/io.h>
#include <linux/hdq.h>

/* -------------------------------------------------------------------------------
 * GTA02 FIQ related
 *
 * Calls into vibrator and hdq and based on the return values
 * determines if we the FIQ source be kept alive
 */

#define DIVISOR_FROM_US(x) ((x) << 3)

#ifdef CONFIG_HDQ_GPIO_BITBANG
#define FIQ_DIVISOR_HDQ DIVISOR_FROM_US(HDQ_SAMPLE_PERIOD_US)
extern int hdq_fiq_handler(void);
#endif

/* Global data related to our fiq source */
static uint32_t gta02_fiq_ack_mask;
static struct s3c2410_pwm gta02_fiq_pwm_timer;
static uint16_t gta02_fiq_timer_index;
static int gta02_fiq_irq;

void gta02_fiq_handler(void)
{
	uint16_t divisor = 0xffff;

	/* disable further timer interrupts if nobody has any work
	 * or adjust rate according to who still has work
	 *
	 * CAUTION: it means forground code must disable FIQ around
	 * its own non-atomic S3C2410_INTMSK changes... not common
	 * thankfully and taken care of by the fiq-basis patch
	 */

#ifdef CONFIG_HDQ_GPIO_BITBANG
	if (hdq_fiq_handler())
		divisor = FIQ_DIVISOR_HDQ;
#endif

	if (divisor == 0xffff) /* mask the fiq irq source */
		__raw_writel(__raw_readl(S3C2410_INTMSK) | gta02_fiq_ack_mask,
				S3C2410_INTMSK);
	else /* still working, maybe at a different rate */
		__raw_writel(divisor, S3C2410_TCNTB(gta02_fiq_timer_index));

	__raw_writel(gta02_fiq_ack_mask, S3C2410_SRCPND);
}

void gta02_fiq_kick(void)
{
	unsigned long flags;
	uint32_t tcon;

	/* we have to take care about FIQ because this modification is
	 * non-atomic, FIQ could come in after the read and before the
	 * writeback and its changes to the register would be lost
	 * (platform INTMSK mod code is taken care of already)
	 */
	local_save_flags(flags);
	local_fiq_disable();
	/* allow FIQs to resume */
	__raw_writel(__raw_readl(S3C2410_INTMSK) &
			~(1 << (gta02_fiq_irq - S3C2410_CPUIRQ_OFFSET)),
			S3C2410_INTMSK);
	tcon = __raw_readl(S3C2410_TCON) & ~S3C2410_TCON_T3START;
	/* fake the timer to a count of 1 */
	__raw_writel(1, S3C2410_TCNTB(gta02_fiq_timer_index));
	__raw_writel(tcon | S3C2410_TCON_T3MANUALUPD, S3C2410_TCON);
	__raw_writel(tcon | S3C2410_TCON_T3MANUALUPD | S3C2410_TCON_T3START,
			S3C2410_TCON);
	__raw_writel(tcon | S3C2410_TCON_T3START, S3C2410_TCON);
	local_irq_restore(flags);
}

int gta02_fiq_enable(void)
{
	int irq_index_fiq = IRQ_TIMER3;
	int rc = 0;

	local_fiq_disable();

	gta02_fiq_irq = irq_index_fiq;
	gta02_fiq_ack_mask = 1 << (irq_index_fiq - S3C2410_CPUIRQ_OFFSET);
	gta02_fiq_timer_index = (irq_index_fiq - IRQ_TIMER0);

	/* set up the timer to operate as a pwm device */

	rc = s3c2410_pwm_init(&gta02_fiq_pwm_timer);
	if (rc)
		goto bail;

	gta02_fiq_pwm_timer.timerid = PWM0 + gta02_fiq_timer_index;
	gta02_fiq_pwm_timer.prescaler = (6 - 1) / 2;
	gta02_fiq_pwm_timer.divider = S3C2410_TCFG1_MUX3_DIV2;
	/* default rate == ~32us */
	gta02_fiq_pwm_timer.counter = gta02_fiq_pwm_timer.comparer = 3000;

	rc = s3c2410_pwm_enable(&gta02_fiq_pwm_timer);
	if (rc)
		goto bail;

	s3c2410_pwm_start(&gta02_fiq_pwm_timer);

	/* let our selected interrupt be a magic FIQ interrupt */
	__raw_writel(gta02_fiq_ack_mask, S3C2410_INTMOD);

	/* it's ready to go as soon as we unmask the source in S3C2410_INTMSK */
	local_fiq_enable();

	set_fiq_c_handler(gta02_fiq_handler);

	return 0;

bail:
	printk(KERN_ERR "Could not initialize FIQ for GTA02: %d\n", rc);

	return rc;
}

void gta02_fiq_disable(void)
{
	__raw_writel(0, S3C2410_INTMOD);
	local_fiq_disable();
	gta02_fiq_irq = 0; /* no active source interrupt now either */

}
/* -------------------- /GTA02 FIQ Handler ------------------------------------- */
