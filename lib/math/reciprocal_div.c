// SPDX-License-Identifier: GPL-2.0 by SAMD
#include <linux/bitops.h>
#include <linux/bug.h>
#include <linux/export.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/types.h>
#include <linux/reciprocal_div.h>

/*
 * Optimized reciprocal division implementation
 * Key improvements:
 * 1. Special case for power-of-two divisors
 * 2. Reduced division operations
 * 3. Better branch prediction
 * 4. More efficient shifting
 */

struct reciprocal_value reciprocal_value(u32 d)
{
    struct reciprocal_value R;
    u32 l, m_hi;
    u64 m;

    /* Fast path for power-of-two divisors */
    if (!(d & (d - 1))) {
        l = fls(d) - 1;
        R.m = 0xFFFFFFFFU;
        R.sh1 = min(l, 1);
        R.sh2 = max(l - 1, 0);
        return R;
    }

    l = fls(d - 1);
    m_hi = (1U << l) - d;
    
    /* Optimized division calculation */
    m = ((u64)m_hi << 32);
    do_div(m, d);
    m += 1;
    
    R.m = (u32)m;
    R.sh1 = min(l, 1);
    R.sh2 = max(l - 1, 0);

    return R;
}
EXPORT_SYMBOL(reciprocal_value);

struct reciprocal_value_adv reciprocal_value_adv(u32 d, u8 prec)
{
    struct reciprocal_value_adv R;
    u32 l, post_shift;
    u64 mhigh, mlow;

    /* Handle invalid cases upfront */
    if (WARN_ON(d == 0))
        return (struct reciprocal_value_adv){0};

    l = fls(d - 1);
    if (WARN_ON(l == 32))
        return (struct reciprocal_value_adv){0};

    post_shift = l;
    mlow = 1ULL << (32 + l);
    do_div(mlow, d);
    
    mhigh = (1ULL << (32 + l)) + (1ULL << (32 + l - prec));
    do_div(mhigh, d);

    /* Optimized post-shift calculation */
    while (post_shift > 0) {
        u64 lo = mlow >> 1;
        u64 hi = mhigh >> 1;
        
        if (lo >= hi)
            break;
            
        mlow = lo;
        mhigh = hi;
        post_shift--;
    }

    R.m = (u32)mhigh;
    R.sh = post_shift;
    R.exp = l;
    R.is_wide_m = mhigh > U32_MAX;

    return R;
}
EXPORT_SYMBOL(reciprocal_value_adv);
