/*
 * $Id$
 *
 * Copyright (C) 2003 ETC s.r.o.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Written by Marcel Telka <marcel@telka.sk>, 2003.
 *
 */

#ifndef CHAIN_H
#define	CHAIN_H

#include "cable.h"
#include "part.h"

typedef struct {
	int state;
	parts_t *parts;
	cable_t *cable;
} chain_t;

chain_t *chain_alloc( void );
void chain_free( chain_t *chain );
int chain_connect( chain_t *chain, cable_t *cable, unsigned int port );
void chain_clock( chain_t *chain, int tms, int tdi );
int chain_set_trst( chain_t *chain, int trst );
int chain_get_trst( chain_t *chain );
void chain_shift_instructions( chain_t *chain );
void chain_shift_data_registers( chain_t *chain );

typedef struct {
	chain_t **chains;
	int size;			/* allocated chains array size */
} chains_t;

#endif /* CHAIN_H */
