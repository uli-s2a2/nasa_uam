#ifndef NAVIGATOR__PLANNER_EXCEPTIONS_HPP_
#define NAVIGATOR__PLANNER_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace navigator
{

class NavigatorException : public std::runtime_error
{
public:
	explicit NavigatorException(const std::string & description)
			: std::runtime_error(description) {}
};

class InvalidNavigator : public NavigatorException
{
public:
	explicit InvalidNavigator(const std::string & description)
			: NavigatorException(description) {}
};

class InvalidTransition : public NavigatorException
{
public:
	explicit InvalidTransition(const std::string & description)
			: NavigatorException(description) {}
};

class NavigatorModeActivationFailed : public NavigatorException
{
public:
	explicit NavigatorModeActivationFailed(const std::string & description)
			: NavigatorException(description) {}
};


}  // namespace navigator

#endif  // NAVIGATOR_PLANNER_EXCEPTIONS_HPP_