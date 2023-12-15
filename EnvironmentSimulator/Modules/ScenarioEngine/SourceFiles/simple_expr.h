/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

// expression implementation is based on https://github.com/zserge/expr

#ifdef __cplusplus
extern "C"
{
#endif

    // Repeat type declarations from expr.h
    // avoid including that module to keep it isolated from static code analysis

    enum expr_return_type
    {
        EXPR_RETURN_UNDEFINED,
        EXPR_RETURN_DOUBLE,
        EXPR_RETURN_STRING,
    };

    typedef struct expr_return_struct
    {
        enum expr_return_type type;
        double                _double;
        struct
        {
            char*  string;
            size_t len;
        } _string;
    } ExprReturnStruct;

    /**
     * Evaluate expression
     * @param str Expression
     * @return evaluated resulting value as float
     */
    ExprReturnStruct eval_expr(const char* str);

    /**
     * Clear allocated data in the result struct
     * @param rs The result struct
     */
    void clear_expr_result(ExprReturnStruct* rs);

#ifdef __cplusplus
} /* extern "C" */
#endif
