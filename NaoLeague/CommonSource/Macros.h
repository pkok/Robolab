//11-10-2012 version Tijmen Blankevoort

#include <iostream>

#ifndef __COUNTER__
#define __Counter -1
#endif

#ifndef __FUNCTION__
#define __FUNCTION__ "{}"
#endif

#define __COMPANYNAME__ "Intelligent Robotics Lab"
#define __PROJECTNAME__ "IRL"
#define VERSIONNR __DATE__



// Use this for error messages that will most likely crash the code
// Persists in release
#ifndef T_ERROR
#define T_ERROR(msg) \
	{\
		std::cout	<< "ERROR:" __COUNTER__ " in " __PROJECTNAME__ << "." << VERSIONNR ":\n" \
					<< msg << std::endl \
					<< "Please contact the administrator at " __COMPANYNAME__ ".\n"; \
		std::getchar(); \
		std::exit(EXIT_FAILURE);  \
	}
// Use this for error messages that warn the user of a critical misusage,
// but which will not crash the code
// Persists in release
#endif
#ifndef WARNING
#define WARNING(msg) \
	{\
		std::cout	<< "Warning from " __PROJECTNAME__ ": " msg << std::endl;\
	}
#endif

//use this for general assertions, persists in release
#ifndef ASSERT
#define ASSERT(condition,msg) \
	{\
		if(!(condition)) \
		{\
			std::cout	<< "ASSERT ERROR in: " __FUNCTION__ " in " __FILE__ "(" << __LINE__ << "):\n" \
			<< msg << std::endl; std::getchar(); std::exit(EXIT_FAILURE); \
		}else{}  \
	} //else is needed against else hijacking
#endif
//warning assert, used for non crashing problems in the code when condition is not satisfied
//persists in release

#ifndef WASSERT
#define WASSERT(condition,msg) \
	{\
		if(!(condition)) \
		{\
			std::cout	<< "ASSERT ERROR in: " __FUNCTION__ " in " __FILE__ "(" << __LINE__ << "):\n" \
			<< msg << std::endl; \
		}else{}  \
	} //else is needed against else hijacking
#endif
// Use this while debugging for full
// will only print in debug version

#ifdef _DEBUG 
#ifndef DASSERT
#define DASSERT(condition,msg) \
	{\
		if(!(condition)) \
		{\
			std::cout	<< "ASSERT ERROR in: " __FUNCTION__ " in " __FILE__ "(" << __LINE__ << "):\n" \
			<< msg << std::endl; \
		}else{}  \
	} //else is needed against else hijacking

#endif
#else
#ifndef DASSERT
#define DASSERT(condition,msg) {}
#endif
#endif



