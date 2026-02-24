#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdint.h>
#include <vector>

extern "C" {
#include "ddsli_core.h"
}

static ddsli_ctx_t *ctx = NULL;

/* ===================== OPEN ===================== */

static PyObject *
py_ddsli_open(PyObject *self, PyObject *args)
{
    const char *port = NULL;
    unsigned long baud = 0;

    if (ctx) {
        PyErr_SetString(PyExc_RuntimeError, "DDSLI already open");
        return NULL;
    }

    if (!PyArg_ParseTuple(args, "|sk", &port, &baud))
        return NULL;

    ctx = ddsli_open(port, (speed_t)baud);
    if (!ctx) {
        PyErr_SetString(PyExc_RuntimeError, "Failed to open DDSLI");
        return NULL;
    }

    Py_RETURN_NONE;
}

/* ===================== CONTROL ===================== */

static PyObject *
py_ddsli_toggle_out_stream(PyObject *self, PyObject *args)
{
    if (!ctx) {
        PyErr_SetString(PyExc_RuntimeError, "DDSLI not open");
        return NULL;
    }

    ddsli_toggle_out_stream(ctx);
    Py_RETURN_NONE;
}

static PyObject *
py_ddsli_send_cmd(PyObject *self, PyObject *args)
{
    const char *cmd;
    Py_ssize_t len;

    if (!ctx) {
        PyErr_SetString(PyExc_RuntimeError, "DDSLI not open");
        return NULL;
    }

    if (!PyArg_ParseTuple(args, "s#", &cmd, &len))
        return NULL;

    int ret = ddsli_send_cmd(ctx, (char *)cmd, (uint32_t)len);
    return PyLong_FromLong(ret);
}

/* ===================== READ BLOCKS ===================== */

static PyObject *
py_ddsli_read_blocks(PyObject *self, PyObject *args)
{
    int n = 1;

    if (!ctx) {
        PyErr_SetString(PyExc_RuntimeError, "DDSLI not open");
        return NULL;
    }

    if (!PyArg_ParseTuple(args, "|i", &n))
        return NULL;

    if (n <= 0) {
        PyErr_SetString(PyExc_ValueError, "n must be positive");
        return NULL;
    }

    PyObject *list = PyList_New(0);

    for (int i = 0; i < n; i++) {
        ddsli_block_t b;
        if (!ddsli_read_block(ctx, &b))
            break;

        PyObject *m = PyDict_New();

        /* frequency sub-dict */
        PyObject *freq = PyDict_New();
        PyDict_SetItemString(freq, "phase",
            PyLong_FromLongLong(b.phase));
        PyDict_SetItemString(freq, "phase_inc",
            PyLong_FromLongLong(b.phase_inc));
        PyDict_SetItemString(freq, "phase_inc_delta",
            PyLong_FromLongLong(b.phase_inc_delta));

        PyDict_SetItemString(m, "frequency", freq);
        Py_DECREF(freq);

        /* chA / chB */
        PyObject *chA = PyList_New(3);
        PyObject *chB = PyList_New(3);
        for (int k = 0; k < 3; k++) {
            PyList_SetItem(chA, k, PyFloat_FromDouble(b.chA[k]));
            PyList_SetItem(chB, k, PyFloat_FromDouble(b.chB[k]));
        }

        PyDict_SetItemString(m, "chA", chA);
        PyDict_SetItemString(m, "chB", chB);
        Py_DECREF(chA);
        Py_DECREF(chB);

        PyList_Append(list, m);
        Py_DECREF(m);
    }

    return list;
}

/* ===================== READ BLOCKS (MATRIX) ===================== */

static PyObject *
py_ddsli_read_blocks_matrix(PyObject *self, PyObject *args)
{
    if (!ctx) {
        PyErr_SetString(PyExc_RuntimeError, "DDSLI not open");
        return NULL;
    }

    std::vector<ddsli_block_t> blocks;
    ddsli_block_t b;

    while (ddsli_read_block(ctx, &b))
        blocks.push_back(b);

    size_t n = blocks.size();
    if (n == 0)
        return PyDict_New();

    PyObject *phase = PyList_New(n);
    PyObject *phase_inc = PyList_New(n);
    PyObject *phase_inc_delta = PyList_New(n);

    PyObject *chA = PyList_New(3);
    PyObject *chB = PyList_New(3);
    for (int k = 0; k < 3; k++) {
        PyList_SetItem(chA, k, PyList_New(n));
        PyList_SetItem(chB, k, PyList_New(n));
    }

    for (size_t i = 0; i < n; i++) {
        PyList_SetItem(phase, i,
            PyLong_FromLongLong(blocks[i].phase));
        PyList_SetItem(phase_inc, i,
            PyLong_FromLongLong(blocks[i].phase_inc));
        PyList_SetItem(phase_inc_delta, i,
            PyLong_FromLongLong(blocks[i].phase_inc_delta));

        for (int k = 0; k < 3; k++) {
            PyList_SetItem(PyList_GetItem(chA, k), i,
                PyFloat_FromDouble(blocks[i].chA[k]));
            PyList_SetItem(PyList_GetItem(chB, k), i,
                PyFloat_FromDouble(blocks[i].chB[k]));
        }
    }

    PyObject *out = PyDict_New();
    PyDict_SetItemString(out, "phase", phase);
    PyDict_SetItemString(out, "phase_inc", phase_inc);
    PyDict_SetItemString(out, "phase_inc_delta", phase_inc_delta);
    PyDict_SetItemString(out, "chA", chA);
    PyDict_SetItemString(out, "chB", chB);

    Py_DECREF(phase);
    Py_DECREF(phase_inc);
    Py_DECREF(phase_inc_delta);
    Py_DECREF(chA);
    Py_DECREF(chB);

    return out;
}

/* ===================== READ ADC ===================== */

static PyObject *
py_ddsli_read_adc(PyObject *self, PyObject *args)
{
    if (!ctx) {
        PyErr_SetString(PyExc_RuntimeError, "DDSLI not open");
        return NULL;
    }

    const int samples = ADC_PACKET_BYTES / 2;
    std::vector<uint16_t> buf(samples);

    if (!ddsli_read_adc_packet(ctx, buf.data()))
        Py_RETURN_NONE;

    PyObject *out = PyList_New(samples);
    for (int i = 0; i < samples; i++)
        PyList_SetItem(out, i, PyLong_FromUnsignedLong(buf[i]));

    return out;
}

/* ===================== CLOSE ===================== */

static PyObject *
py_ddsli_close(PyObject *self, PyObject *args)
{
    if (ctx) {
        ddsli_close(ctx);
        ctx = NULL;
    }
    Py_RETURN_NONE;
}

/* ===================== MODULE ===================== */

static PyMethodDef ddsli_methods[] = {
    {"open", py_ddsli_open, METH_VARARGS, NULL},
    {"toggle_out_stream", py_ddsli_toggle_out_stream, METH_NOARGS, NULL},
    {"send_cmd", py_ddsli_send_cmd, METH_VARARGS, NULL},
    {"read_blocks", py_ddsli_read_blocks, METH_VARARGS, NULL},
    {"read_blocks_matrix", py_ddsli_read_blocks_matrix, METH_NOARGS, NULL},
    {"read_adc", py_ddsli_read_adc, METH_NOARGS, NULL},
    {"close", py_ddsli_close, METH_NOARGS, NULL},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef ddsli_module = {
    PyModuleDef_HEAD_INIT,
    "ddsli",
    NULL,
    -1,
    ddsli_methods
};

PyMODINIT_FUNC
PyInit_ddsli(void)
{
    return PyModule_Create(&ddsli_module);
}
