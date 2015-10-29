/**
 * \file	list_port.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	29/10/2015
 * \copyright Copyright (c) 2015 S.O.N.I.A.. All rights reserved.
 * Use of this source code is governed by the MIT license that can be
 * found in the LICENSE file.
 */

#include <lib_atlas/macros.h>

#if defined(OS_LINUX)
#include <lib_atlas/sys/details/list_ports_linux.h>
#elif (OS_DARWIN)
#include <lib_atlas/sys/details/list_ports_osx.h>
#else
#error "OS not supported."
#endif
