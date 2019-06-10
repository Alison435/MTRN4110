// Made a start on units.cpp. Very confusing and not sure at all
// which functions we are meant to implement or not implement :(
// Will try and test these somehow...

// Implementation of the units.h files
// Required for digital IO, motor, wheel etc

#include "Arduino.h"
#include "units.h"

using namespace units;

// For reference:
//using value_type = value_t; eg unsigned long
//private:->value_type value_;
//using unit_type = derived_units_name; eg microseconds

template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator+ (unit_type rhs) const -> unit_type
{

    unit_type summed_unit;
    unit_type lhs;

    summed_unit = lhs + rhs;

    return summed_unit;

}

    /**
     * \brief The operator- subtract a units object from another object of the
     * same type. \param rhs is amount to subtract. \return a new object storing
     * the result.
     */
template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator- (unit_type rhs) const -> unit_type
{

    unit_type lhs;
    unit_type subtracted_unit = lhs - rhs;

    return subtracted_unit;

}

template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator* (value_type rhs) const -> unit_type
{

}

template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator/ (value_type rhs) const -> unit_type
{

}

/**
     * \brief The operator< compares two units of the same type.
     * \param rhs is the right hand side value
     * \return true if lhs < rhs
     */
template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator< (unit_type rhs) const -> bool
{

}

    /**
     * \brief The operator<= compares two units of the same type.
     * \param rhs is the right hand side value
     * \return true if lhs <= rhs
     */
template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator<= (unit_type rhs) const -> bool
{

}

    /**
     * \brief The operator> compares two units of the same type.
     * \param rhs is the right hand side value
     * \return true if lhs > rhs
     */
template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator> (unit_type rhs) const -> bool
{

}

    /**
     * \brief The operator>= compares two units of the same type.
     * \param rhs is the right hand side value
     * \return true if lhs >= rhs
     */
template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator>= (unit_type rhs) const -> bool
{

}

    /**
     * \brief The operator== compares two units of the same type.
     * \param rhs is the right hand side value
     * \return true if lhs == rhs
     */
template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator== (unit_type rhs) const -> bool
{

}

    /**
     * \brief The operator!= compares two units of the same type.
     * \param rhs is the right hand side value
     * \return true if lhs != rhs
     */
template <typename derived_units_name, typename value_t = double>
auto base_unit<derived_units_name,value_t>::operator!= (unit_type rhs) const -> bool
{

}

// microseconds and millseconds are classes inherited from
// base class (appear to be non-template classes)

// microseconds
/**
     * \brief The now method returns the number of microseconds since the
     * program started. Overflow approximately every 70 minutes
     * \return microseconds since start of program
     */
static auto microseconds::now () noexcept -> microseconds
{

    microseconds micro_elapsed;
    micro_elapsed = micros();

    return micro_elapsed;

}

// milliseconds
/**
     * \brief The now  method returns the number of microseconds since the
     * program started. Overflow approximately every 50 days.
     * \return milliseconds since start of program
     */
static auto milliseconds::now () noexcept -> milliseconds
{

    milliseconds millis_elapsed;
    millis_elapsed = millis();

    return millis_elapsed;

}

// These are just in the units namespace

    /**
 * \brief The sleep_for method pause the program for period of time.
 * \param period is the number of milliseconds to pause the program.
 */
auto units::sleep_for (milliseconds period) -> void
{

    milliseconds timer;

    while (true)
    {
        if ((millis() - timer.now()) > period)
        {
            break;
        }
    }
}

/**
 * \brief The sleep_for method pause the program for period of time.
 * \param period is the number of microseconds to pause the program.
 */
auto units::sleep_for (microseconds period) -> void
{

    microseconds timer;

    while (true)
    {
        if ((micros() - timer.now()) > period)
        {
            break;
        }
    }
}

/**
 * \brief The sleep_until pause the program until a specific time since start of
 * program
 * \param wake_time is the time to resume execution in milliseconds.
 */
auto units::sleep_until (milliseconds wake_time) -> void
{

}

/**
 * \brief The sleep_until pause the program until a specific time since start of
 * program
 * \param wake_time is the time to resume execution in microseconds.
 */
auto units::sleep_until (microseconds wake_time) -> void
{

}
