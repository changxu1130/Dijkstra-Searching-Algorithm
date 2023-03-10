#include "extensions.h"
#include "options.h"

//////////////////////////////////////////////////////////////////////
// For sorting colors

int color_features_compare(const void* vptr_a, const void* vptr_b) {

	const color_features_t* a = (const color_features_t*)vptr_a;
	const color_features_t* b = (const color_features_t*)vptr_b;

	int u = cmp(a->user_index, b->user_index);
	if (u) { return u; }

	int w = cmp(a->wall_dist[0], b->wall_dist[0]);
	if (w) { return w; }

	int g = -cmp(a->wall_dist[1], b->wall_dist[1]);
	if (g) { return g; }

	return -cmp(a->min_dist, b->min_dist);

}

//////////////////////////////////////////////////////////////////////
// Place the game colors into a set order

void game_order_colors(game_info_t* info,
                       game_state_t* state) {

	if (g_options.order_random) {
    
		srand(now() * 1e6);
    
		for (size_t i=info->num_colors-1; i>0; --i) {
			size_t j = rand() % (i+1);
			int tmp = info->color_order[i];
			info->color_order[i] = info->color_order[j];
			info->color_order[j] = tmp;
		}

	} else { // not random

		color_features_t cf[MAX_COLORS];
		memset(cf, 0, sizeof(cf));

		for (size_t color=0; color<info->num_colors; ++color) {
			cf[color].index = color;
			cf[color].user_index = MAX_COLORS;
		}
    

		for (size_t color=0; color<info->num_colors; ++color) {
			
			int x[2], y[2];
			
			for (int i=0; i<2; ++i) {
				pos_get_coords(state->pos[color], x+i, y+i);
				cf[color].wall_dist[i] = get_wall_dist(info, x[i], y[i]);
			}

			int dx = abs(x[1]-x[0]);
			int dy = abs(y[1]-y[0]);
			
			cf[color].min_dist = dx + dy;
			
		

		}


		qsort(cf, info->num_colors, sizeof(color_features_t),
		      color_features_compare);

		for (size_t i=0; i<info->num_colors; ++i) {
			info->color_order[i] = cf[i].index;
		}
    
	}

	if (!g_options.display_quiet) {

		printf("\n************************************************"
		       "\n*               Branching Order                *\n");
		if (g_options.order_most_constrained) {
			printf("* Will choose color by most constrained\n");
		} else {
			printf("* Will choose colors in order: ");
			for (size_t i=0; i<info->num_colors; ++i) {
				int color = info->color_order[i];
				printf("%s", color_name_str(info, color));
			}
			printf("\n");
		}
		printf ("*************************************************\n\n");

	}

}



//////////////////////////////////////////////////////////////////////
// Check for dead-end regions of freespace where there is no way to
// put an active path into and out of it. Any freespace node which
// has only one free neighbor represents such a dead end. For the
// purposes of this check, cur and goal positions count as "free".
// return 1 if deadend has found, 0 otherwise

int game_check_deadends(const game_info_t* info,
                        const game_state_t* state) {
	/*
	detect not free cells 
	(e.g. not a path, not a goal, not an initial state)
	 */
	
	int num_free;
	pos_t last_move = (state->pos)[state->last_color];

	//void pos_get_coords(pos_t p, int* x, int* y)
	int x, y, lm_nb_x, lm_nb_y;
	pos_get_coords(last_move, &x, &y);


	for (int lm_nb_dir = DIR_LEFT; lm_nb_dir <= DIR_DOWN; ++lm_nb_dir) {
		pos_t lm_nb_pos = offset_pos(info, x, y, lm_nb_dir);
		pos_get_coords(lm_nb_pos, &lm_nb_x, &lm_nb_y);

		if(lm_nb_pos == INVALID_POS){
			continue;
		}
		
		num_free = 0;
		//for each cell find its neighbor
		for (int dir = DIR_LEFT; dir <= DIR_DOWN; ++dir) {
			pos_t neighbor_pos = offset_pos(info, lm_nb_x, lm_nb_y, dir);

			if(neighbor_pos == INVALID_POS){
				continue;
			}

			//free cells 
			//1. cell type == free
			if (neighbor_pos != INVALID_POS &&
				(cell_get_type(state->cells[neighbor_pos]) == TYPE_FREE)) {
				++num_free;
			}

			// path 
			//2. cell type == path && cell pos is in head && cell color is incomplete
			else if (neighbor_pos != INVALID_POS &&
					cell_get_type(state->cells[neighbor_pos]) == TYPE_PATH  && 
					(neighbor_pos == 
							state->pos[(cell_get_color(state->cells[neighbor_pos]))]) &&
					( !(state->completed & 
					(1 << (cell_get_color(state->cells[neighbor_pos]))))) 
					){
				++num_free;
			}

			//initial 
			//3. cell type == init && cell pos is in head
			else if (neighbor_pos != INVALID_POS &&
					cell_get_type(state->cells[neighbor_pos]) == TYPE_INIT  && 
					(neighbor_pos == state->pos[(cell_get_color(state->cells[neighbor_pos]))]) 
					){
				++num_free;
			}

			//goal 
			//4. cell type == goal && cell color is incomplete
			else if (neighbor_pos != INVALID_POS &&
					cell_get_type(state->cells[neighbor_pos]) == TYPE_GOAL  
					&&
					( !(state->completed & 
					(1 << (cell_get_color(state->cells[neighbor_pos]))))) 
					){
				++num_free;
			}
			//check neighbour's neighbour

			int nb_num_free;
			int nb_x, nb_y;
			pos_get_coords(neighbor_pos, &nb_x, &nb_y);
			if	(cell_get_type(state->cells[neighbor_pos]) == TYPE_FREE){
				nb_num_free = 0;

				for (int nb_dir = DIR_LEFT; nb_dir <= DIR_DOWN; ++nb_dir) {
					pos_t nb_neighbor_pos = offset_pos(info, nb_x, nb_y, nb_dir);

					//free cells 
					//1. cell type == free
					if (nb_neighbor_pos != INVALID_POS &&
						(cell_get_type(state->cells[nb_neighbor_pos]) == TYPE_FREE)) {
						++nb_num_free;
					}

					// path 
					//2. cell type == path && cell pos is in head && cell color is incomplete
					else if (nb_neighbor_pos != INVALID_POS &&
							cell_get_type(state->cells[nb_neighbor_pos]) == TYPE_PATH  && 
							(nb_neighbor_pos == 
									state->pos[(cell_get_color(state->cells[nb_neighbor_pos]))]) &&
							( !(state->completed & 
							(1 << (cell_get_color(state->cells[nb_neighbor_pos]))))) 
							){
						++nb_num_free;
					}

					//initial 
					//3. cell type == init && cell pos is in head
					else if (nb_neighbor_pos != INVALID_POS &&
							cell_get_type(state->cells[nb_neighbor_pos]) == TYPE_INIT  && 
							(nb_neighbor_pos == 
									state->pos[(cell_get_color(state->cells[nb_neighbor_pos]))]) 
							){
						++nb_num_free;
					}

					//goal 
					//4. cell type == goal && cell color is incomplete
					else if (nb_neighbor_pos != INVALID_POS &&
							cell_get_type(state->cells[nb_neighbor_pos]) == TYPE_GOAL  
							&&
							( !(state->completed & 
							(1 << (cell_get_color(state->cells[nb_neighbor_pos]))))) 
							){
						++nb_num_free;
					}
				}

				//found a deadend, return straight away
				if(nb_num_free <= 1){
					return 1;
				} 
			}
		}

		//found a deadend, return straight away
		if((num_free <= 1) && (cell_get_type(state->cells[lm_nb_pos]) == TYPE_FREE)){
			return 1;
		} 
		
				
	}	

	return 0; 
}

