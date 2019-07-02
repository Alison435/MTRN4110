// Implementation of the units.h files
// Required for digital IO, motor, wheel etc

#include "Arduino.h"
#include "units.h"

using namespace units;

//using unit_type = derived_units_name;
//using value_type = value_t;

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator+ (unit_type rhs) const -> unit_type
{

    unit_type sum = this->count() + rhs.count();
    return sum;

}

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator- (unit_type rhs) const -> unit_type
{

    unit_type val = this->count() - rhs.count();
    return val;

}

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator* (value_type rhs) const -> unit_type
{

    unit_type val = this->count() * rhs.count();
    return val;

}

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator/ (value_type rhs) const -> unit_type
{

    unit_type val = this->count / rhs.count();
    return val;

}

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator< (unit_type rhs) const -> bool
{

    if (this->count() < rhs.count())
    {
        return true;
    }
    else
    {
        return false;
    }

}

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator> (unit_type rhs) const -> bool
{

    if (this->count() > rhs.count())
        {
            return true;
        }
        else
        {
            return false;
        }

}

/*
template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator<= (unit_type rhs) const -> bool
{

}

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator>= (unit_type rhs) const -> bool
{

}

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator== (unit_type rhs) const -> bool
{

}

template <typename derived_units_name,typename value_t>
auto base_unit<derived_units_name,value_t>::operator!= (unit_type rhs) const -> bool
{

}
*/
