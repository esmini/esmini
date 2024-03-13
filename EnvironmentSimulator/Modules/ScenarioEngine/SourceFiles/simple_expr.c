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

#include "expr.h"
#include "simple_expr.h"

// Custom function that returns the floor of its argument
static double round_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return rint(a);
}

// Custom function that returns the floor of its argument
static double floor_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return floor(a);
}

// Custom function that returns the floor of its argument
static double ceil_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return ceil(a);
}

// Custom function that returns the floor of its argument
static double sqrt_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return sqrt(a);
}

// Custom function that returns first argument raised to the power of the second argument
static double pow_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    double b = expr_eval(&vec_nth(args, 1));
    return pow(a, b);
}

// Custom function that returns first argument raised to the power of the second argument
static double sin_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return sin(a);
}

static double cos_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return cos(a);
}

static double tan_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return tan(a);
}

static double asin_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return asin(a);
}

static double acos_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return acos(a);
}

static double atan_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return atan(a);
}

static double sign_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return (a < 0 ? -1 : 1);
}

static double abs_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    return fabs(a);
}

static double max_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    double b = expr_eval(&vec_nth(args, 1));
    return (b > a ? b : a);
}

static double min_(struct expr_func* f, vec_expr_t* args, void* c)
{
    (void)f;
    (void)c;
    double a = expr_eval(&vec_nth(args, 0));
    double b = expr_eval(&vec_nth(args, 1));
    return (b < a ? b : a);
}

static struct expr_func user_funcs[] = {
    // specified in OpenSCENARIO >= 1.1
    {"round", round_, NULL, 0},
    {"floor", floor_, NULL, 0},
    {"ceil", ceil_, NULL, 0},
    {"sqrt", sqrt_, NULL, 0},
    {"pow", pow_, NULL, 0},
    // additional expressions not specified in OSC <=1.2
    // may not work in other OpenSCENARIO compliant tools
    {"sin", sin_, NULL, 0},
    {"cos", cos_, NULL, 0},
    {"tan", tan_, NULL, 0},
    {"asin", asin_, NULL, 0},
    {"acos", acos_, NULL, 0},
    {"atan", atan_, NULL, 0},
    {"sign", sign_, NULL, 0},
    {"abs", abs_, NULL, 0},
    {"max", max_, NULL, 0},
    {"min", min_, NULL, 0},
    {NULL, NULL, NULL, 0},
};

ExprReturnStruct eval_expr(const char* str)
{
    ExprReturnStruct     rs   = {EXPR_RETURN_UNDEFINED, NAN, {0, 0}};
    struct expr_var_list vars = {0};
    struct expr*         e    = expr_create(str, strlen(str), &vars, user_funcs);
    if (e == 0 || e->type == OP_UNKNOWN)
    {
        return rs;
    }
    else if (e->type == OP_STR)
    {
        rs.type           = EXPR_RETURN_STRING;
        rs._double        = NAN;
        rs._string.string = malloc(e->param.str.len + 1);
        if (rs._string.string != 0)
        {
            memcpy(rs._string.string, e->param.str.value, e->param.str.len + 1);
            rs._string.len = e->param.str.len;
        }
    }
    else
    {
        double value = expr_eval(e);
        if (!isnan(value))
        {
            rs.type    = EXPR_RETURN_DOUBLE;
            rs._double = value;
        }
    }

    expr_destroy(e, 0);

    return rs;
}

void clear_expr_result(ExprReturnStruct* rs)
{
    if (rs->_string.string != 0)
    {
        if (rs->_string.len == 0)
        {
            printf("simple_expr: Warning: clearing an 0 length expression string");
        }
        free(rs->_string.string);
        rs->_string.len = 0;
    }
}

#ifdef __cplusplus
} /* extern "C" */
#endif
