/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#ifndef rw_common_macros_HPP
#define rw_common_macros_HPP

/**
 * @file macros.hpp
 */

#include "Exception.hpp"
#include "Log.hpp"
#include "Message.hpp"
#include "IOUtil.hpp"

#include <sstream>
#include <iostream>

/** @addtogroup common */
/*@{*/

/**
 * @brief Throw an exception with message \b ostreamExpression.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
 * \code
 *  RW_THROW("The value of x is " << x);
 * \endcode
 *
 * Exception messages can be intercepted via exceptionLog().
 */
#define RW_THROW(ostreamExpression) do { int RW__line = __LINE__;           \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());  \
    if( rw::common::Log::log().isEnabled(rw::common::Log::Debug) ) \
		rw::common::Log::debugLog().write(RW__message);        \
    throw rw::common::Exception(RW__message);                               \
} while (0)
// We use the weird RW__ names to (hopefully) avoid name crashes.



/**
 * @brief Throw an exception with the specified id and message \b ostreamExpression.
 *
 * \b id is the id of the exception and * \b ostreamExpression is an expression that 
 * is fed to an output stream. Example:
 * \code
 *  RW_THROW("The value of x is " << x);
 * \endcode
 *
 * Exception data can be intercepted via exceptionLog().
 */
#define RW_THROW2(id, ostreamExpression) do { int RW__line = __LINE__;           \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());  \
    rw::common::Exception exp(id, RW__message);                               \
    if( rw::common::Log::log().isEnabled(rw::common::Log::Error) ) \
    	rw::common::Log::errorLog().write(exp.what());                              \
    throw exp;                                                                  \
} while (0)
// We use the weird RW__ names to (hopefully) avoid name crashes.


/**
 * @brief Emit a warning.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
RW_WARN("The value of x is " << x << ". x should be less than zero.");
\endcode
 *
 * Warning messages can be intercepted via warningLog().
 */
#define RW_WARN(ostreamExpression) do { int RW__line = __LINE__;            \
    std::stringstream RW__stream;                                           \
    RW__stream << ostreamExpression;                                        \
    rw::common::Message RW__message(__FILE__, RW__line, RW__stream.str());  \
    if( rw::common::Log::log().isEnabled(rw::common::Log::Warning) ) \
    	rw::common::Log::warningLog() << RW__message << std::endl;      \
} while (0)

/**
 * @brief Emit debug message.
 *
 * \b ostreamExpression is an expression that is fed to an output stream. Example:
\code
RW_DEBUG("The value of x is " << x << ". x should be less than zero.");
 *
\endcode
 * Debug messages can be intercepted via debugLog().
 */
#ifdef RW_DEBUG_ENABLE
#define RW_DEBUG(ostreamExpression) do { int RW__line = __LINE__;                                               \
    if( rw::common::Log::log().isEnabled(rw::common::Log::Debug) ) { \
    	rw::common::Log::debugLog() << __FILE__ << ":" << RW__line << " " << ostreamExpression << std::endl;        \
    } \
} while (0)
#else
#define RW_DEBUG(ostreamExpression)
#endif

/**
 * @brief For internal use only.
 */
#define RW_ASSERT_IMPL(e, ostreamExpression, file, line) \
    do { std::stringstream RW__stream;                                   \
        RW__stream << ostreamExpression;                                 \
        ((e) ? (void)0 : rw::common::IOUtil::rwAssert(RW__stream.str().c_str(), file, line)); \
    } while(0)
    
/**
 * @brief RobWork assertions.
 *
 * RW_ASSERT() is an assertion macro in the style of assert(). RW_ASSERT() can
 * be enabled by compiling with RW_ENABLE_ASSERT defined. Otherwise RW_ASSERT()
 * is enabled if NDBUG is not defined.
 *
 * You should prefer RW_ASSERT() to assert() everywhere with a possible
 * exception being (unsafe) access of arrays and a few other places where you
 * are sure that a run time sanity check will be a performance issue.
 */
#ifdef RW_ENABLE_ASSERT
#  define RW_ASSERT(e) RW_ASSERT_IMPL(e, std::string(#e), __FILE__, __LINE__)
#else
#  ifdef NDEBUG
#    define RW_ASSERT(e)
#  else
#    define RW_ASSERT(e) RW_ASSERT_IMPL(e, std::string(#e), __FILE__, __LINE__)
#  endif
#endif

/**
 * @brief RobWork assertions with user-friendly messages.
 *
 * RW_ASSERT_MSG() is an assertion macro similar to RW_ASSERT(). RW_ASSERT_MSG() can
 * be enabled by compiling with RW_ENABLE_ASSERT defined. Otherwise RW_ASSERT_MSG()
 * is enabled if NDBUG is not defined.
 *
 * RW_ASSERT_MSG() takes one additional argument, allowing printing user-friendly
 * error message if assertion fails.
 */
#ifdef RW_ENABLE_ASSERT
#  define RW_ASSERT_MSG(e, msg) RW_ASSERT_IMPL(e, msg, __FILE__, __LINE__)
#else
#  ifdef NDEBUG
#    define RW_ASSERT_MSG(e, msg)
#  else
#    define RW_ASSERT_MSG(e, msg) RW_ASSERT_IMPL(e, msg, __FILE__, __LINE__)
#  endif
#endif

/**
 * @brief Writes \b ostreamExpression to log with LogIndex \b id.
 *
 * \b id be the log level of type rw::common::Log::LogIndex.
 *
 * \b ostreamExpression is an expression that is fed to an output stream.
 *
 * Example:
 * \code
 * int x = 1;
 * RW_LOG(rw::common::Log::Debug, "Warning: The value of x " << x << " is too small");
 * \endcode
 *
 * @param id [in] log level to write to.
 * @param ostreamExpression [in] Stream expression which should be written to the log
 */
#define RW_LOG(id, ostreamExpression) do { rw::common::Log::log().get(id) << ostreamExpression << std::endl; } while (0)

/**
 * @brief Writes \b ostreamExpression to error log.
 * @param ostreamExpression [in] Stream expression which should be written to the log
 */
#define RW_LOG_ERROR(ostreamExpression) RW_LOG(rw::common::Log::Error, ostreamExpression)

/**
 * @brief Writes \b ostreamExpression to warning log.
 * @param ostreamExpression [in] Stream expression which should be written to the log
 */
#define RW_LOG_WARNING(ostreamExpression) RW_LOG(rw::common::Log::Warning, ostreamExpression)

/**
 * @brief Writes \b ostreamExpression to debug log.
 * @param ostreamExpression [in] Stream expression which should be written to the log
 */
#define RW_LOG_DEBUG(ostreamExpression) RW_LOG(rw::common::Log::Debug, ostreamExpression)

/**
 * @brief Writes \b ostreamExpression to info log.
 * @param ostreamExpression [in] Stream expression which should be written to the log
 */
#define RW_LOG_INFO(ostreamExpression) RW_LOG(rw::common::Log::Info, ostreamExpression)

/**
 * @brief Convenient convertion of a string expression into a Message.
 * @param ostreamExpression [in] stream expression which should be converted to a Message.
 */
#define RW_MSG(ostreamExpression) (Message(__FILE__, __LINE__)<<ostreamExpression)

/**
 * @brief enables the use of a \b robwork namespace
 */
#define USE_ROBWORK_NAMESPACE \
	namespace rw { namespace proximity {}} \
	namespace rw { namespace common {}} \
	namespace rw { namespace control {}} \
	namespace rw { namespace geometry {}} \
	namespace rw { namespace interpolator {}} \
	namespace rw { namespace trajectory {}} \
	namespace rw { namespace invkin {}} \
	namespace rw { namespace kinematics {}} \
	namespace rw { namespace math {}} \
	namespace rw { namespace models {}} \
	namespace rw { namespace pathplanning {}} \
	namespace rw { namespace graspplanning {}} \
	namespace rw { namespace sensor {}} \
	namespace rw { namespace loaders {}} \
	namespace rw { namespace graphics {}} \
	namespace rwlibs { namespace algorithms {}} \
    namespace rwlibs { namespace devices {}} \
    namespace rwlibs { namespace dll {}} \
    namespace rwlibs { namespace opengl {}} \
    namespace rwlibs { namespace io {}} \
    namespace rwlibs { namespace lua {}} \
    namespace rwlibs { namespace os {}} \
    namespace rwlibs { namespace pathoptimization {}} \
    namespace rwlibs { namespace pathplanners {}} \
    namespace rwlibs { namespace proximitystrategies {}} \
    namespace rwlibs { namespace sensors {}} \
    namespace rwlibs { namespace task {}} \
    namespace rwlibs { namespace simulation {}} \
	namespace robwork \
	{ \
		using namespace rw; \
		using namespace rw::proximity; \
		using namespace rw::common; \
		using namespace rw::control; \
		using namespace rw::geometry; \
		using namespace rw::interpolator; \
		using namespace rw::trajectory; \
		using namespace rw::invkin; \
		using namespace rw::kinematics; \
		using namespace rw::math; \
		using namespace rw::models; \
		using namespace rw::pathplanning; \
		using namespace rw::graspplanning; \
		using namespace rw::sensor; \
		using namespace rw::loaders; \
		using namespace rw::graphics; \
        using namespace rwlibs; \
        using namespace rwlibs::algorithms; \
        using namespace rwlibs::devices; \
        using namespace rwlibs::dll; \
        using namespace rwlibs::opengl; \
        using namespace rwlibs::io; \
        using namespace rwlibs::lua; \
        using namespace rwlibs::os; \
        using namespace rwlibs::pathoptimization; \
        using namespace rwlibs::pathplanners; \
        using namespace rwlibs::proximitystrategies; \
        using namespace rwlibs::sensors; \
        using namespace rwlibs::task; \
        using namespace rwlibs::simulation; \
    }

/*@}*/

#endif // end include guard
