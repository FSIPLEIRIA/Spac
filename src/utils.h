/** ==========================================
* @title: Utilities
* @class: This class serves to aid the other classes in the project by providing pre-processed constants and functionalities
* @author: João Vieira
* @date:   2023-06-14
========================================== **/

#ifndef UTILS_H_
#define UTILS_H_


/**
* 
*/


/**
* @brief The distance between the front and rear axles
*/
#ifdef VOIDSTER
	#define WHEELBASE 1.550f
	#define MAX_STEERING (2*M_PI/9)
	#define TERMINAL_SPEED 144.0f
	//TODO if mission have different safety and objectives then there should be more macros
	#define DISTANCE_MAX 80.0f
	
	
#endif
#ifdef ANT
	#define WHEELBASE 1.550f
	#define MAX_STEERING (2*M_PI/9) // TODO check it
#endif



#endif // UTILS_H_