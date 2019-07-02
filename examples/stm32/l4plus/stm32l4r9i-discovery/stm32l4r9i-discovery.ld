/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2019 Guillaume Meunier <guillaume.meunier@centraliens.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Linker script for ST STM32L4R9IDISCOVERY (STM32L4R9AI, 2048K flash, 640K RAM). */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 2048K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 640K
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

