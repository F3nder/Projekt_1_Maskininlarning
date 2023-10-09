/********************************************************************************
 * @brief Implementation details for the container library.
 ********************************************************************************/
#pragma once

#include <stdlib.h>

namespace yrgo {
namespace container {
namespace detail {
namespace {

/********************************************************************************
 * @brief Allocates a new block of on the heap.
 *
 * @param size
 *        The size of the allocated block, i.e. the number of elements it 
 *        can hold.
 * @return
 *        A pointer to the allocated block at success, else a null pointer.
 ********************************************************************************/
template <typename T>
inline T* New(const size_t size) {
    return static_cast<T*>(malloc(sizeof(T) * size));
}

/********************************************************************************
 * @brief Resizes referenced heap allocated block via reallocation.
 *
 * @param block
 *        The block to resize.
 * @param new_size
 *        The new size of the allocated block, i.e. the number of elements it 
 *        can hold after reallocation.
 * @return
 *        A pointer to the resized block at success, else a null pointer.
 ********************************************************************************/
template <typename T>
inline T* Resize(T* block, const size_t new_size) {
    return static_cast<T*>(realloc(block, sizeof(T) * new_size));
}

/********************************************************************************
 * @brief Delete heap allocated block via deallocation. The pointer to the
 *        block is set to null after deallocation.
 *
 * @param block
 *        Reference to the block to delete.
 ********************************************************************************/
template <typename T>
inline void Delete(T* &block) {
    free(block);
    block = nullptr;
}

/********************************************************************************
 * @brief Moves memory from specified source to a copy. The source no longer
 *        has ownership after the move is performed, i.e. the source is emptied.
 *
 * @param source
 *        Reference to the source whose memory is moved.
 * @return 
 *        The copy of the source.
 ********************************************************************************/
template <typename T>
inline T Move(T&& source) {
    T copy{source};
    source = {};
    return copy;
}

} /* namespace */
} /* namespace detail */
} /* namespace container */
} /* namespace yrgo */