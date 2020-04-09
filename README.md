# nav

Prototype navigation filter for Lunar missions

* [Open Lunar Foundation](https://www.openlunar.org/)

## Description

This is currently work-in-progress code! It is designed mainly for
testing flight computers. It compiles and runs but it does not
work. There are certainly major problems. Don't use it.

## Requirements

* C++ compiler (clang, gcc)
* CMake
* Eigen 3

## Usage

Clone this repository:

    git clone https://github.com/openlunar/nav.git

Change directories into the nav folder. Create a `build/` directory,
change into it, and then do

    cmake ..
    make

## Developers

If you find a bug or wish to make a contribution, use the project's
[github issue tracker](https://github.com/openlunar/nav/issues).

## License

Copyright (c) 2019--2020, John O. "Juno" Woods and Open Lunar Foundation.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.