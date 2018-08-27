/* -*- c++ -*- */

#define SOAPY_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "soapy_swig_doc.i"

%{
#include "soapy/soapy_source.h"
#include "soapy/soapy_sink.h"
%}


%include "soapy/soapy_source.h"
GR_SWIG_BLOCK_MAGIC2(soapy, soapy_source);
%include "soapy/soapy_sink.h"
GR_SWIG_BLOCK_MAGIC2(soapy, soapy_sink);
