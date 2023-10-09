/********************************************************************************
 * @brief Implementation of static arrays of any data type.
 ********************************************************************************/
#pragma once

#include <stdlib.h>

namespace yrgo {
namespace container {

/********************************************************************************
 * @brief Class for implementation of static arrays.
 ********************************************************************************/
template <typename T, size_t size>
class Array {
    static_assert(size > 0, "Static array size cannot be set to 0!");
  public:

    /********************************************************************************
     * @brief Default constructor, creates array of specified size.
     ********************************************************************************/
    Array(void) = default;

    /********************************************************************************
     * @brief Creates array containing referenced values.
     *
     * @param values
     *        Reference to the values to store in newly created array.
     ********************************************************************************/
    Array(const T (&values)[size]) noexcept {
        Copy(values);
    }

    /********************************************************************************
     * @brief Creates array as a copy of referenced source.
     *
     * @param source
     *        Reference to array whose content is copied to the new array.
     ********************************************************************************/
    template <size_t num_values>
    Array(const Array<T, num_values>& source) noexcept {
        Copy(source);
    }

    /********************************************************************************
     * @brief Default destructor. Statically allocated memory is freed automatically
     *        when the array goes out of scope, so nothing needs to be done.
     ********************************************************************************/
    ~Array(void) = default;

    /********************************************************************************
     * @brief Returns reference to the element at specified index in referenced 
     *        array. An exception occurs if the index falls outside the array.
     *
     * @param index
     *        Index to searched element.
     * @return
     *        A reference to the element at specified index.
     ********************************************************************************/
    T& operator[](const size_t index) noexcept {
        return data_[index];
    }

    /********************************************************************************
     * @brief Returns reference to the element at specified index in referenced 
     *        array. An exception occurs if the index falls outside the array.
     *
     * @param index
     *        Index to searched element.
     * @return
     *        A reference to the element at specified index.
     ********************************************************************************/
    const T& operator[](const size_t index) const noexcept {
        return data_[index];
    }

     /********************************************************************************
     * @brief Copies referenced values to assigned array. 
     *
     * @param values
     *        Referenced values to copy.
     * @return
     *        A reference to assigned array.     
     ********************************************************************************/
    template <size_t num_values>
    Array& operator=(const T (&values)[num_values]) noexcept {
        Copy(values);
        return *this;
    }

    /********************************************************************************
     * @brief Copies the content of referenced array to assigned array. 
     *
     * @param source
     *        Reference to array containing the the values to copy.
     * @return
     *        A reference to assigned array.     
     ********************************************************************************/
    template <size_t num_values>
    Array& operator=(const Array<T, num_values>& source) noexcept {
        Copy(source);
        return *this;
    }

    /********************************************************************************
     * @brief Adds referenced values to the back of assigned array.
     *
     * @param values
     *        Reference to the values to add to assigned array.
     * @return
     *        A reference to assigned array.     
     ********************************************************************************/
    template <size_t num_values>
    Array& operator+=(const T (&values)[num_values]) noexcept {
        Copy(values, num_values);
        return *this;
    }

    /********************************************************************************
     * @brief Adds values from referenced array to the back of assigned array.
     *
     * @param source
     *        Reference to array containing the the values to add.
     * @return
     *        A reference to assigned array.     
     ********************************************************************************/
    template <size_t num_values>
    Array& operator+=(const Array<T, num_values>& source) noexcept {
        Copy(source, num_values);
        return *this;
    }

    /********************************************************************************
     * @brief Returns a pointer to the data stored in specified array.
     *
     * @return
     *        A pointer to allocated memory block containing stored elements.
     ********************************************************************************/
    T* Data(void) noexcept {
        return data_;
    }

    /********************************************************************************
     * @brief Returns a pointer to the data stored in specified array.
     *
     * @return
     *        A pointer to allocated memory block containing stored elements.
     ********************************************************************************/
    T* Data(void) const noexcept {
        return data_;
    }

    /********************************************************************************
     * @brief Returns the size of referenced array, i.e. the number of elements 
     *        it can hold.
     *
     * @return
     *        The size of the array as the number of elements it can hold.
     ********************************************************************************/
    size_t Size(void) const noexcept {
        return size;
    }

    /********************************************************************************
     * @brief Returns the start address of referenced array.
     *
     * @return
     *        A pointer to the first element of referenced array.
     ********************************************************************************/
    T* begin(void) noexcept {
        return data_;
    }

    /********************************************************************************
     * @brief Returns the start address of referenced array.
     *
     * @return
     *        A pointer to the first element of referenced array.
     ********************************************************************************/
    const T* begin(void) const noexcept {
        return data_;
    }

    /********************************************************************************
     * @brief Returns the end address of referenced array.
     *
     * @return
     *        A pointer to the address after the last element of referenced array.
     ********************************************************************************/
    T* end(void) noexcept {
        return data_ + size;
    }

    /********************************************************************************
     * @brief Returns the end address of referenced array.
     *
     * @return
     *        A pointer to the address after the last element of referenced array.
     ********************************************************************************/
    const T* end(void) const noexcept {
        return data_ + size;
    }

    /********************************************************************************
     * @brief Returns the address of the last element stored in referenced array.
     *
     * @return
     *        A pointer to the last element stored in referenced array.
     ********************************************************************************/
    T* last(void) noexcept {
        return end() - 1;
    }

   /********************************************************************************
     * @brief Returns the address of the last element stored in referenced array.
     *
     * @return
     *        A pointer to the last element stored in referenced array.
     ********************************************************************************/
    const T* last(void) const noexcept {
        return end() - 1;
    }

    /********************************************************************************
     * @brief Clears content of referenced array.
     ********************************************************************************/
    void Clear(void) noexcept {
        for (auto& i : *this) {
            i = static_cast<T>(0);
        }
    }

  private:
    T data_[size]{}; /* Static array. */

    /********************************************************************************
     * @brief Copies referenced values and assigns to referenced array. An offset
     *        can be specified to assign the values further back in referenced array. 
     *        The offset should be set to the index of the first assigned value.
     *
     * @param source
     *        Reference to array containing values to assign.
     * @param offset
     *        Offset to assign values from a specified starting index.
     ********************************************************************************/
    template <size_t num_values>
    void Copy(const T (&values)[num_values], const size_t offset = 0) {
        for (size_t i{}; i + offset < size && i < num_values; ++i) {
            data_[offset + i] = values[i];
        }
    }

    /********************************************************************************
     * @brief Copies values from referenced source array. An offset can be 
     *        specified to assign the values further back in referenced array. 
     *        The offset should be set to the index of the first assigned value.
     *
     * @param values
     *        Referenced values to assign.
     * @param offset
     *        Offset to assign values from a specified starting index.
     ********************************************************************************/
    template <size_t num_values>
    void Copy(const Array<T, num_values>& source, const size_t offset = 0) {
        for (size_t i{}; i + offset < size && i < source.Size(); ++i) {
            data_[offset + i] = source[i];
        }
    }
};

} /* namespace container */
} /* namespace yrgo */