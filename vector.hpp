/********************************************************************************
 * @brief Implementation of dynamic vectors of any data type.
 ********************************************************************************/
#pragma once

#include <container.hpp>

namespace yrgo {
namespace container {

/********************************************************************************
 * @brief Class for implementation of dynamic vectors.
 ********************************************************************************/
template <typename T>
class Vector {
  public:

    /********************************************************************************
     * @brief Default constructor, creates empty vector.
     ********************************************************************************/
    Vector(void) noexcept = default;

    /********************************************************************************
     * @brief Creates vector of specified size.
     *
     * @param size
     *        The size of the vector, i.e. the number of elements it can hold.
     ********************************************************************************/
    Vector(const size_t size) noexcept {
        Resize(size);
    }

    /********************************************************************************
     * @brief Creates vector containing referenced values.
     *
     * @param values
     *        Reference to the values to store in newly created vector.
     ********************************************************************************/
    template <size_t size>
    Vector(const T (&values)[size]) noexcept {
        if (Resize(size)) {
            Assign(values);
        }
    }

    /********************************************************************************
     * @brief Creates vector as a copy of referenced source.
     *
     * @param source
     *        Reference to vector whose content is copied to the new vector.
     ********************************************************************************/
    Vector(const Vector& source) noexcept { 
        Copy(source); 
    }

    /********************************************************************************
     * @brief Move constructor, moves content from referenced source to assigned
     *        vector. The source is emptied after the move operation is performed.
     *
     * @param source
     *        Reference to vector whose content is moved to assigned vector.
     ********************************************************************************/
    Vector(Vector&& source) noexcept {
        data_ = source.data_;
        size_ = source.size_;
        source.data_ = nullptr;
        source.size_ = 0;
    }

    /********************************************************************************
     * @brief Destructor, clears memory allocated for referenced vector.
     *
     * @param source
     *        Reference to vector whose content is moved to assigned vector.
     ********************************************************************************/
    ~Vector(void) noexcept { Clear(); }

    /********************************************************************************
     * @brief Index operator, returns reference to the element at specified index 
     *        in referenced vector. 
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
     * @brief Index operator, returns reference to the element at specified index 
     *        in referenced vector. 
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
     * @brief Assignment operator, copies the content of referenced vector to
     *        assigned vector. Previous values are cleared before copying.
     *
     * @param values
     *        Referenced values to copy.    
     ********************************************************************************/
    template <size_t size>
    void operator=(const T (&values)[size]) noexcept {
        Clear();
        Copy(values);
    }

    /********************************************************************************
     * @brief Assignment operator, copies the content of referenced vector to 
     *        assigned vector. Previous values are cleared before copying.
     *
     * @param source
     *        Reference to vector containing the the values to add.    
     ********************************************************************************/
    void operator=(const Vector& source) noexcept {
        Clear();
        Copy(source);
    }

    /********************************************************************************
     * @brief Addition operator, pushes referenced values to the back of the vector.
     *
     * @param values
     *        Reference to the values to add.
     ********************************************************************************/
    template <size_t size>
    void operator+=(const T (&values)[size]) noexcept {
        AddValues(values);
    }

    /********************************************************************************
     * @brief Adds values from referenced vector to the back of assigned vector.
     *
     * @param source
     *        Reference to vector containing the the values to add.
     ********************************************************************************/
    void operator+=(const Vector& source) noexcept {
        AddValues(source);
    }

    /********************************************************************************
     * @brief Returns a pointer to the data stored in specified vector.
     *
     * @return
     *        A pointer to allocated memory block containing stored elements.
     ********************************************************************************/
    T* Data(void) noexcept{
        return data_;
    }

    /********************************************************************************
     * @brief Returns a pointer to the data stored in specified vector.
     *
     * @return
     *        A pointer to allocated memory block containing stored elements.
     ********************************************************************************/
    const T* Data(void) const noexcept {
        return data_;
    }

    /********************************************************************************
     * @brief Returns the size of referenced vector, i.e. the number of elements 
     *        it can hold.
     *
     * @return
     *        The size of the vector as the number of elements it can hold.
     ********************************************************************************/
    size_t Size(void) const noexcept {
        return size_;
    }

    /********************************************************************************
     * @brief Checks if referenced vector is empty.
     *
     * @return
     *        True if referenced vector is empty, else false.
     ********************************************************************************/
    bool Empty(void) const noexcept {
        return size_ == 0 ? true : false;
    }

    /********************************************************************************
     * @brief Returns the start address of referenced vector.
     *
     * @return
     *        A pointer to the first element of referenced vector.
     ********************************************************************************/
    T* begin(void) noexcept { return data_; }

    /********************************************************************************
     * @brief Returns the start address of referenced vector.
     *
     * @return
     *        A pointer to the first element of referenced vector.
     ********************************************************************************/
    const T* begin(void) const noexcept { return data_; }

    /********************************************************************************
     * @brief Returns the end address of referenced vector.
     *
     * @return
     *        A pointer to the address after the last element of referenced vector.
     ********************************************************************************/
    T* end(void) noexcept { return data_ + size_; }

    /********************************************************************************
     * @brief Returns the end address of referenced vector.
     *
     * @return
     *        A pointer to the address after the last element of referenced vector.
     ********************************************************************************/
    const T* end(void) const noexcept { return data_ + size_; }

    /********************************************************************************
     * @brief Returns the address of the last element stored in referenced vector.
     *
     * @return
     *        A pointer to the last element stored in referenced vector.
     ********************************************************************************/
    T* last(void) noexcept { return size_ > 0 ? end() - 1 : nullptr; }

    /********************************************************************************
     * @brief Returns the address of the last element stored in referenced vector.
     *
     * @return
     *        A pointer to the last element stored in referenced vector.
     ********************************************************************************/
    const T* last(void) const noexcept { return size_ > 0 ? end() - 1 : nullptr; }

    /********************************************************************************
     * @brief Clears content of referenced vector by deallocating memory on the
     *        heap. All member variables are reset to starting values.
     ********************************************************************************/
    void Clear(void) noexcept {
        detail::Delete<T>(data_);
        size_ = 0;
    }

    /********************************************************************************
     * @brief Resizes referenced vector to specified new size via reallocation
     *        of heap allocated memory block. The memory block is unchanged if
     *        the memory allocation fails.
     *
     * @param new_size
     *        The new size of the vector after reallocation.
     * @return
     *        True if the reallocation succeeded, else false.
     ********************************************************************************/
    bool Resize(const size_t new_size) noexcept {
        auto copy{detail::Resize<T>(data_, new_size)};
        if (copy == nullptr) return false;
        data_ = copy;
        size_ = new_size;
        return true;
    }

    /********************************************************************************
     * @brief Pushes new value to the back of referenced vector.
     *
     * @param value
     *        Reference to the new value to push to the vector.
     * @return
     *        True if the value was pushed to the back of the vector, else false.
     ********************************************************************************/
    bool PushBack(const T& value) noexcept {
        if (Resize(size_ + 1)) {
            data_[size_ - 1] = value;
            return true;
        } else {
            return false;
        }
    }

    /******************************************************************************** 
     * @brief Pops value at the back of referenced vector via reallocation.
     *
     * @return
     *        True if the last value of the vector was popped, else false.
     ********************************************************************************/
    bool PopBack(void) noexcept {
        if (size_ <= 1) {
            Clear();
            return true;
        } else {
            return Resize(size_ - 1);
        }
    }

  private:
    T* data_{nullptr}; /* Pointer to dynamically allocated memory block. */
    size_t size_{};    /* The size of the vector in number of elements it can hold. */

    /********************************************************************************
     * @brief Copies the content of referenced source. All previous elements are
     *        either removed or overwritten.
     *
     * @param source
     *        Reference to vector whose content is copied.
     * @return
     *        True if the content of the source vector was copied, else false.
     ********************************************************************************/
    bool Copy(const Vector& source) noexcept {
        if (Resize(source.size_)) {
            Assign(source);
            return true;
        } else {
            return false;
        }
    }

    /********************************************************************************
     * @brief Assigns referenced values without changing the vector size. An offset 
     *        can be specified to assign the values further back in referenced vector. 
     *        The offset should be set to the index of the first assigned value.
     *
     * @param values
     *        Referenced values to assign.
     * @param offset
     *        Offset to assign values from a specified starting index.
     ********************************************************************************/
    template <size_t size>
    void Assign(const T (&values)[size], const size_t offset = 0) noexcept {
        for (size_t i{}; i < size && offset + i < size_; ++i) {
            data_[offset + i] = values[i];
        }
    }

    /********************************************************************************
     * @brief Assigns values from referenced source vector without changing the
     *        vector size. An offset can be specified to assign the values further
     *        back in referenced vector. The offset should be set to the index of
     *        the first assigned value.
     *
     * @param source
     *        Reference to vector containing values to assign.
     * @param offset
     *        Offset to assign values from a specified starting index.
     ********************************************************************************/
    void Assign(const Vector& source, const size_t offset = 0) noexcept {
        for (size_t i{}; offset + i < size_ && i < source.size_; ++i) {
            data_[offset + i] = source.data_[i];
        }
    }

    /********************************************************************************
     * @brief Adds referenced values to the back of the vector.
     *
     * @param values
     *        Reference to values to copy and add to the back of the vector.
     * @return
     *        True if the values were added, else false.
     ********************************************************************************/
    template <size_t size>
    bool AddValues(const T (&values)[size]) noexcept {
        const auto offset{size_};
        if (Resize(size_ + size)) {
            Assign(values, offset);
            return true;
        } else {
            return false;
        }
    }

    /********************************************************************************
     * @brief Adds values from referenced source vector to the back of the vector.
     *
     * @param source
     *        Reference to vector whose content is copied and added to the vector.
     * @return
     *        True if the values were added, else false.
     ********************************************************************************/
    bool AddValues(const Vector& source) noexcept {
        const auto offset{size_};
        if (Resize(size_ + source.size_)) {
            Assign(source, offset);
            return true;
        } else {
            return false;
        }
    }
};

} /* namespace container */
} /* namespace yrgo */