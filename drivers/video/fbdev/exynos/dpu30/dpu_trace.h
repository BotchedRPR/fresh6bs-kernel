/* SPDX-License-Identifier: GPL-2.0 */
/*
 * DPU trace support
 *
 * Copyright (C) 2020 Google, Inc.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM dpu

#if !defined(_DPU_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _DPU_TRACE_H_

#include <linux/tracepoint.h>

TRACE_EVENT(tracing_mark_write,

        TP_PROTO(char *ev),
        TP_ARGS(ev),
        TP_STRUCT__entry(
                            __string(event, ev)
        ),
        TP_fast_assign(
                            __assign_str(event, ev);
        ),
        TP_printk("%s", __get_str(event))
    );

#endif /* _DPU_TRACE_H_ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .

#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE dpu_trace

/* This part must be outside protection */
#include <trace/define_trace.h>
