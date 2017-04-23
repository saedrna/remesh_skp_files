//
//  TypedValue.cpp
//
//  Sketchup C++ Wrapper for C API
//  Copyright (C) 2016  Hidetomo (Tom) Kaneko
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <cassert>

#include "TypedValue.hpp"
#include "Color.hpp"
#include "Geometry.hpp"
#include "String.hpp"

namespace CW {

TypedValue::TypedValue():
    m_typed_value(create_typed_value())
{}

TypedValue::TypedValue(SUTypedValueRef typed_val):
    m_typed_value(typed_val)
{}

TypedValue::TypedValue(const char chars[]):
    m_typed_value(create_typed_value()) {
    this->string_value(String(chars));
}

/** Copy constructor */
TypedValue::TypedValue(const TypedValue &other):
    TypedValue() {
    // Redo copy assignment operator
    (*this) = other;
}


TypedValue &TypedValue::operator= (const TypedValue &other) {
    switch (other.get_type()) {
    case SUTypedValueType_Array: {
        std::vector<TypedValue> other_typed = other.typed_value_array();
        typed_value_array(other_typed);
    }
    break;
    case SUTypedValueType_Bool:
        bool_value(other.bool_value());
        break;
    case SUTypedValueType_Byte:
        byte_value(other.byte_value());
        break;
    case SUTypedValueType_Color:
        color_value(other.color_value());
        break;
    case SUTypedValueType_Double:
        double_value(other.double_value());
        break;
    case SUTypedValueType_Empty:
        // Empty TypedValues are all the same.
        break;
    case SUTypedValueType_Float:
        float_value(other.float_value());
        break;
    case SUTypedValueType_Int32:
        int32_value(other.int32_value());
        break;
    case SUTypedValueType_Short:
        // TODO
        throw "this part hasn't been made yet.";
        break;
    case SUTypedValueType_String:
        string_value(other.string_value());
        break;
    case SUTypedValueType_Time:
        time_value(other.time_value());
        break;
    case SUTypedValueType_Vector3D:
        vector_value(other.vector_value());
        break;
    }
    return (*this);
}


SUTypedValueRef TypedValue::ref() const {
    return m_typed_value;
}

TypedValue::operator SUTypedValueRef() {
    return ref();
}

TypedValue::operator SUTypedValueRef *() {
    return &m_typed_value;
}

bool TypedValue::operator!() const {
    if (SUIsValid(m_typed_value)) {
        return false;
    }
    return true;
}

bool TypedValue::empty() const {
    if (SUIsInvalid(m_typed_value) ||
            get_type() == SUTypedValueType::SUTypedValueType_Empty) {
        return true;
    }
    return false;
}



/*
* Static member function
*/
SUTypedValueRef TypedValue::create_typed_value() {
    SUTypedValueRef typed_value = SU_INVALID;
    SU_RESULT res = SUTypedValueCreate(&typed_value);
    assert(res == SU_ERROR_NONE);
    return typed_value;
}

TypedValue::~TypedValue() {
    if (SUIsValid(m_typed_value)) {
        SU_RESULT res = SUTypedValueRelease(&m_typed_value);
        assert(res == SU_ERROR_NONE);
    }
}


SUTypedValueType TypedValue::get_type() const {
    if (!(*this)) {
        return SUTypedValueType::SUTypedValueType_Empty;
    }
    SUTypedValueType type;
    SU_RESULT res = SUTypedValueGetType(m_typed_value, &type);
    assert(res == SU_ERROR_NONE);
    return type;
}


char TypedValue::byte_value() const {
    if (!(*this)) {
        return 0;
    }
    char byte_val;
    SU_RESULT res = SUTypedValueGetByte(m_typed_value, &byte_val);
    assert(res == SU_ERROR_NONE);
    return byte_val;
}


TypedValue &TypedValue::byte_value(const char byte_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SU_RESULT res = SUTypedValueSetByte(m_typed_value, byte_val);
    assert(res == SU_ERROR_NONE);
    return *this;
}


TypedValue::operator char() const {
    return byte_value();
}


int16_t TypedValue::int16_value() const {
    if (!(*this)) {
        return 0;
    }
    int16_t int16_val;
    SU_RESULT res = SUTypedValueGetInt16(m_typed_value, &int16_val);
    assert(res == SU_ERROR_NONE);
    return int16_val;
}

TypedValue &TypedValue::int16_value(const int16_t int16_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SU_RESULT res = SUTypedValueSetInt16(m_typed_value, int16_val);
    assert(res == SU_ERROR_NONE);
    return *this;
}
TypedValue::operator int16_t() const {
    return int16_value();
}


int32_t TypedValue::int32_value() const {
    if (!(*this)) {
        return 0;
    }
    int32_t int32_val;
    SU_RESULT res = SUTypedValueGetInt32(m_typed_value, &int32_val);
    assert(res == SU_ERROR_NONE);
    return int32_val;
}

TypedValue &TypedValue::int32_value(const int32_t int32_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SU_RESULT res = SUTypedValueSetInt32(m_typed_value, int32_val);
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue::operator int32_t() const {
    return int32_value();
}


float TypedValue::float_value() const {
    if (!(*this)) {
        return 0;
    }
    float float_val;
    SU_RESULT res = SUTypedValueGetFloat(m_typed_value, &float_val);
    assert(res == SU_ERROR_NONE);
    return float_val;
}

TypedValue &TypedValue::float_value(const float float_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SU_RESULT res = SUTypedValueSetFloat(m_typed_value, float_val);
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue::operator float() const {
    return float_value();
}

double TypedValue::double_value() const {
    if (!(*this)) {
        return 0;
    }
    double double_val;
    SU_RESULT res = SUTypedValueGetDouble(m_typed_value, &double_val);
    assert(res == SU_ERROR_NONE);
    return double_val;
}

TypedValue &TypedValue::double_value(const double double_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SU_RESULT res = SUTypedValueSetDouble(m_typed_value, double_val);
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue::operator double() const {
    return double_value();
}

bool TypedValue::bool_value() const {
    if (!(*this)) {
        return 0;
    }
    bool bool_val;
    SU_RESULT res = SUTypedValueGetBool(m_typed_value, &bool_val);
    assert(res == SU_ERROR_NONE);
    return bool_val;
}

TypedValue &TypedValue::bool_value(const bool bool_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SU_RESULT res = SUTypedValueSetBool(m_typed_value, bool_val);
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue::operator bool() const {
    return bool_value();
}


Color TypedValue::color_value() const {
    if (!(*this)) {
        return Color();
    }
    SUColor color_val;
    SU_RESULT res = SUTypedValueGetColor(m_typed_value, &color_val);
    assert(res == SU_ERROR_NONE);
    return Color(color_val);
}

TypedValue &TypedValue::color_value(const Color &color_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SUColor color = color_val.ref();
    SU_RESULT res = SUTypedValueSetColor(m_typed_value, &color);
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue::operator Color() const {
    return color_value();
}

int64_t TypedValue::time_value() const {
    if (!(*this)) {
        return 0;
    }
    int64_t time_val;
    SU_RESULT res = SUTypedValueGetTime(m_typed_value, &time_val);
    assert(res == SU_ERROR_NONE);
    return time_val;
}

TypedValue &TypedValue::time_value(const int64_t time_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SU_RESULT res = SUTypedValueSetTime(m_typed_value, time_val);
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue::operator int64_t() const {
    return time_value();
}

String TypedValue::string_value() const {
    if (!(*this)) {
        return String();
    }
    String string;
    SUStringRef &string_ref = string;
    SU_RESULT res = SUTypedValueGetString(m_typed_value, &string_ref);
    assert(res == SU_ERROR_NONE);
    return string;
}

TypedValue &TypedValue::string_value(const String &string_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    std::string std_string = string_val;
    SU_RESULT res = SUTypedValueSetString(m_typed_value, std_string.c_str());
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue &TypedValue::string_value(const std::string &string_val) {
    return string_value(String(string_val));
}

TypedValue::operator String() const {
    return string_value();
}
TypedValue::operator std::string() const {
    return string_value();
}


Vector3D TypedValue::vector_value() const {
    if (!(*this)) {
        return Vector3D();
    }
    double vector3d_value[3];
    SU_RESULT res = SUTypedValueGetVector3d(m_typed_value, &vector3d_value[0]);
    assert(res == SU_ERROR_NONE);
    return Vector3D(vector3d_value[0], vector3d_value[1], vector3d_value[2]);
}

TypedValue &TypedValue::vector_value(const Vector3D &vector_val) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    double vector3d_value[3] = {vector_val.x, vector_val.y, vector_val.z};
    SU_RESULT res = SUTypedValueSetVector3d(m_typed_value, &vector3d_value[0]);
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue::operator Vector3D() const {
    return vector_value();
}

std::vector<TypedValue> TypedValue::typed_value_array() const {
    if (!(*this)) {
        return std::vector<TypedValue>();
    }
    size_t count = 0;
    SU_RESULT res = SUTypedValueGetNumArrayItems(m_typed_value, &count);
    assert(res == SU_ERROR_NONE);
    std::vector<SUTypedValueRef> values(count);
    res = SUTypedValueGetArrayItems(m_typed_value, count, &values[0], &count);
    assert(res == SU_ERROR_NONE);
    std::vector<TypedValue> typed_vals;
    typed_vals.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        typed_vals.push_back(TypedValue(values[i]));
    }
    return typed_vals;
}

TypedValue &TypedValue::typed_value_array(std::vector<TypedValue> &typed_val_array) {
    if (!(*this)) {
        m_typed_value = create_typed_value();
    }
    SU_RESULT res = SUTypedValueSetArrayItems(m_typed_value, typed_val_array.size(), typed_val_array[0]);
    assert(res == SU_ERROR_NONE);
    return *this;
}

TypedValue::operator std::vector<TypedValue>() const {
    return typed_value_array();
}


bool operator== (const TypedValue &val1, const TypedValue &val2) {
    // Check validity
    if (SUIsInvalid(val1.m_typed_value) || SUIsInvalid(val1.m_typed_value)) {
        if (SUIsInvalid(val1.m_typed_value) && SUIsInvalid(val1.m_typed_value)) {
            return true;
        }
        return false;
    }
    // Check types
    if (val1.get_type() != val1.get_type()) {
        return false;
    }
    switch (val1.get_type()) {
    case SUTypedValueType_Array:
        if (val1.typed_value_array() == val2.typed_value_array()) {
            return true;
        }
        return false;
        break;
    case SUTypedValueType_Bool:
        if (val1.bool_value() == val2.bool_value()) {
            return true;
        }
        return false;
        break;
    case SUTypedValueType_Byte:
        if (val1.byte_value() == val2.byte_value()) {
            return true;
        }
        return false;
        break;
    case SUTypedValueType_Color:
        if (val1.color_value() == val2.color_value()) {
            return true;
        }
        return false;
        break;
    case SUTypedValueType_Double:
        if (val1.double_value() == val2.double_value()) {
            return true;
        }
        return false;
        break;
    case SUTypedValueType_Empty:
        // Empty TypedValues are all the same.
        return true;
        break;
    case SUTypedValueType_Float:
        if (val1.float_value() == val2.float_value()) {
            return true;
        }
        return false;
        break;
    case SUTypedValueType_Int32:
        if (val1.int32_value() == val2.int32_value()) {
            return true;
        }
        return false;
        break;
    case SUTypedValueType_Short:
        // TODO
        throw "this part hasn't been made yet.";
        break;
    case SUTypedValueType_String:
        if (val1.string_value() == val2.string_value()) {
            return true;
        }
        return false;

        break;
    case SUTypedValueType_Time:
        if (val1.time_value() == val2.time_value()) {
            return true;
        }
        return false;
        break;
    case SUTypedValueType_Vector3D:
        if (val1.vector_value() == val2.vector_value()) {
            return true;
        }
        return false;
        break;
    }
    return false;
}

bool operator!= (const TypedValue &val1, const TypedValue &val2) {
    if (val1 == val2) {
        return false;
    }
    return true;
}


} /* namespace CW */