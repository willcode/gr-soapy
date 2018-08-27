/* -*- c++ -*- */

#define SOAPY_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "soapy_swig_doc.i"

%{
#include "soapy/source.h"
#include "soapy/sink.h"
%}


%include "soapy/source.h"
GR_SWIG_BLOCK_MAGIC2(soapy, source);
%include "soapy/sink.h"
GR_SWIG_BLOCK_MAGIC2(soapy, sink);
