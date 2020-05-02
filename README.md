# gr-soapy: SoapySDR GNU Radio Out-Of-Tree Module
gr-soapy is a GNURadio wrapper for the SoapySDR library.


## Installation

### Requirements
* GNU Radio ( >= 3.8)
* CMake ( >= 3.8)
* G++
* VOLK
* git
* SoapySDR
* python3-soapysdr
* python3-Mako

---
**Note**:
Additionally you must install at least one Soapy SDR module
corresponding to your SDR hardware (e.g.
[soapyrtlsdr](https://github.com/pothosware/SoapyRTLSDR)). See also the
[Pothosware repository](https://github.com/pothosware) for other SDR modules.
---

#### Debian / Ubuntu
```bash
apt-get install \
  libboost-dev \
  libboost-date-time-dev \
  libboost-filesystem-dev \
  libboost-program-options-dev \
  libboost-system-dev \
  libboost-thread-dev \
  libboost-regex-dev \
  libboost-test-dev \
  python3 \
  python3-six \
  python3-mako \
  python3-dev \
  swig \
  cmake \
  gcc \
  gnuradio-dev \
  libsoapysdr-dev \
  libconfig++-dev \
  libgmp-dev \
  liborc-0.4-0 \
  liborc-0.4-dev \
  liborc-0.4-dev-bin \
  git
```

#### openSUSE
```bash
zypper in \
  boost-devel \
  libboost_filesystem-devel \
  libboost_system-devel \
  libboost_thread-devel \
  libboost_program_options-devel \
  libboost_regex-devel \
  libboost_test-devel \
  python3 \
  python3-six \
  python3-Mako \
  python3-devel \
  swig \
  cmake \
  gcc-c++ \
  gcc \
  soapy-sdr \
  soapy-sdr-devel \
  gnuradio \
  gnuradio-devel \
  gmp-devel \
  libmpir-devel \
  liborc-0_4-0 \
  orc \
  log4cpp-devel
```

### Installation from source

The `master` branch of this repository contains the latest development code
of gr-soapy.
If you want to use a stable version, please select one from the available [releases](https://gitlab.com/librespacefoundation/gr-soapy/-/releases).

```bash
git clone https://gitlab.com/librespacefoundation/gr-soapy
cd gr-soapy
mkdir build
cd build
cmake ..
make -j $(nproc --all)
sudo make install
```


If this is the first time you are building the gr-soapy module run

```bash
sudo ldconfig
```


#### Advanced
By default, the **gr-soapy** module will use the default installation prefix.
This highly depends on the Linux distribution. You can use the `CMAKE_INSTALL_PREFIX`
variable to alter the default installation path.
E.g:

```bash
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
```


Another common control option is the library suffix of the Linux distribution.
There are distributions like Fedora, openSUSE, e.t.c that the their 64-bit version
use the `lib64` folder to store the 64-bit versions of their dynamic libraries.
On the other hand, distributions like Ubuntu do the exact opposite. They use
`lib` directory for the libraries of the native architecture and place the 32-bit versions
on the `lib32` directory. In any case the correct library directory suffix
can be specified with the `LIB_SUFFIX` variable. For example:

```bash
cmake -DLIB_SUFFIX=64 -DCMAKE_INSTALL_PREFIX=/usr ..
```

will install the libraries at the `/usr/lib64` directory.

### Coding style
For the C++ code, `gr-soapy` uses a slightly modified version of the 
**Stroustrup** style, which is a nicer adaptation of the well known K&R style.
In addition, we decided to decrease the indentation from 4 to 2 spaces.
This choice was made mainly to avoid braking statements with long namespaces.
We also found ourselves, that with smaller indentation we use more descriptive
variable names, avoiding frustrating abbreviations without phoenixes etc. 

At the root directory of the project there is the `astyle` options 
file `.astylerc` containing the proper configuration.
Developers can import this configuration to their favorite editor. 
In addition the `hooks/pre-commit` file contains a Git hook, 
that can be used to perform before every commit, code style formatting
with `astyle` and the `.astylerc` parameters.
To enable this hook developers should copy the hook at their `.git/hooks` 
directory. 
Failing to comply with the coding style described by the `.astylerc` 
will result to failure of the automated tests running on our CI services. 
So make sure that you either import on your editor the coding style rules 
or use the `pre-commit` Git hook.

## Problems & Support

If you have problems with the installation or usage of gr-soapy feel free to ask
in the [Libre Space Community](https://community.libre.space/). If you think you
found a bug, please open an issue in the
[issue tracker](https://gitlab.com/librespacefoundation/gr-soapy/issues).


## License

&copy; 2018-2020 [Libre Space Foundation](http://libre.space).

Licensed under the [GPLv3](LICENSE).
