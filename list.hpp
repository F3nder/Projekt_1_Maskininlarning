/********************************************************************************
 * @brief Implementation of doubly linked lists of any data type.
 ********************************************************************************/
#pragma once

#include <container.hpp>

namespace yrgo {
namespace container {

/********************************************************************************
 * @brief Class for implementation of doubly linked lists.
 ********************************************************************************/
template <typename T>
class List {
    struct Node; /* Struct for storing data in the linked list. */

  public:
    class Iterator;      /* Class for iterating through nodes in mutable linked lists. */
    class ConstIterator; /* Class for iterating through nodes in constant linked lists. */

    /********************************************************************************
     * @brief Default constructor, creates empty list.
     ********************************************************************************/
    List(void) = default;

    /********************************************************************************
     * @brief Creates list of specified size with specified start value for each
     *        element.
     *
     * @param size
     *        The starting size of the list, i.e. the number of values it can hold.
     * @param start_value
     *        The starting value for each element (default = 0).
     ********************************************************************************/
     List(const size_t size, const T& start_value = static_cast<T>(0)) {
         Resize(size, start_value);
     }

     /********************************************************************************
     * @brief Creates list containing referenced values.
     *
     * @param values
     *        Reference to the values to store in newly created list.
     ********************************************************************************/
    template <size_t size>
    List(const T (&values)[size]) noexcept {
        Copy(values);
    }

    /********************************************************************************
     * @brief Creates list as a copy of referenced source.
     *
     * @param source
     *        Reference to list whose content is copied to the new list.
     ********************************************************************************/
    List(List& source) noexcept {
        Copy(source);
    }

    /********************************************************************************
     * @brief Destructor, clears memory allocated for nodes in referenced list.
     ********************************************************************************/
    ~List(void) noexcept { Clear(); }

    /********************************************************************************
     * @brief Returns reference to the value at specified position in referenced 
     *        list.
     *
     * @param iterator
     *        Reference to iterator pointing at the value to read.
     * @return
     *        A reference to the element at specified position.
     ********************************************************************************/
    T& operator[](Iterator& iterator) noexcept {
        return *iterator;
    }

    /********************************************************************************
     * @brief Returns reference to the element at specified index in referenced 
     *        list.
     *
     * @param iterator
     *        Reference to iterator pointing at the value to read..
     * @return
     *        A reference to the element at specified position.
     ********************************************************************************/
    const T& operator[] (ConstIterator& iterator) const noexcept {
        return *iterator;
    }

    /********************************************************************************
     * @brief Returns the size of referenced list, i.e. the number of elements 
     *        it can hold.
     *
     * @return
     *        The size of the list as the number of elements it can hold.
     ********************************************************************************/
    size_t Size(void) const {
        return size_;
    }

    /********************************************************************************
     * @brief Clears content of referenced list by deallocating memory on the
     *        heap. All member variables are reset to starting values.
     ********************************************************************************/
    void Clear(void) noexcept {
        RemoveAllNodes();
        first_ = nullptr;
        last_ = nullptr;
        size_ = 0;
    }

    /********************************************************************************
     * @brief Checks if referenced list is empty.
     *
     * @return
     *        True if referenced list is empty, else false.
     ********************************************************************************/
    bool Empty(void) const noexcept {
        return size_ == 0 ? true : false;
    }

    /********************************************************************************
     * @brief Returns the address of the first node in referenced list.
     *
     * @return
     *        A pointer to the first node in referenced list.
     ********************************************************************************/
    Iterator begin(void) noexcept {
        return Iterator{first_};
    }

    /********************************************************************************
     * @brief Returns the address of the first node in referenced list.
     *
     * @return
     *        A pointer to the first node in referenced list.
     ********************************************************************************/
    ConstIterator begin(void) const noexcept {
        return ConstIterator{first_};
    }

    /********************************************************************************
     * @brief Returns the ending address of referenced list (which is always null).
     *
     * @return
     *        A pointer to the ending address of referenced list.
     ********************************************************************************/
    Iterator end(void) noexcept {
        return Iterator{nullptr};
    }

     /********************************************************************************
     * @brief Returns the ending address of referenced list (which is always null).
     *
     * @return
     *        A pointer to the ending address of referenced list.
     ********************************************************************************/
    ConstIterator end(void) const noexcept {
        return ConstIterator{nullptr};
    }

    /********************************************************************************
     * @brief Returns the address of the last node in referenced list.
     *
     * @return
     *        A pointer to the last node in referenced list.
     ********************************************************************************/
    Iterator last(void) noexcept {
        return Iterator{last_};
    }

    /********************************************************************************
     * @brief Returns the address of the last node in referenced list.
     *
     * @return
     *        A pointer to the last node in referenced list.
     ********************************************************************************/
    ConstIterator last(void) const noexcept {
        return ConstIterator{last_};
    }

    /********************************************************************************
     * @brief Resizes referenced list to specified new size.
     *
     * @param new_size
     *        The new size of the list after reallocation.
     * @param start_value
     *        The starting value for each element (default = 0).
     * @return
     *        True if the list was resized, else false.
     ********************************************************************************/
    bool Resize(const size_t new_size, const T& start_value = static_cast<T>(0)) noexcept {
        while (size_ < new_size) {
            if (!PushBack(start_value)) {
                return false;
            }
        }
        while (size_ > new_size) {
            PopFront();
        }
        return true;
    }

    /********************************************************************************
     * @brief Inserts value at the front of referenced list via a new node.
     *
     * @param value
     *        Reference to the value to add.
     * @return
     *        True if the value was added, else false.
     ********************************************************************************/
    bool PushFront(const T& value) noexcept {
        auto node1{Node::New(value)};
        if (node1 == nullptr) return false;
        if (size_++ == 0) {
            first_ = node1;
            last_ = node1;
        } else {
            auto node2{first_};
            node1->next = node2;
            node2->previous = node1;
            first_ = node1;
        }
        return true;
    }

    /********************************************************************************
     * @brief Inserts value at the back of referenced list via a new node.
     *
     * @param value
     *        Reference to the value to add.
     * @return
     *        True if the value was added, else false.
     ********************************************************************************/
    bool PushBack(const T& value) noexcept {
        auto node2{Node::New(value)};
        if (node2 == nullptr) return false;   
        if (size_++ == 0) {
            first_ = node2;
            last_ = node2;
        } else {
            auto node1{last_};
            node1->next = node2;
            node2->previous = node1;
            last_ = node2;
        }
        return true;
    }

    /********************************************************************************
     * @brief Inserts value at specified position of referenced list via a node.
     *
     * @param iterator
     *        Reference to iterator pointing where the value is to be inserted.
     * @param value
     *        Reference to the value to add.
     * @return
     *        True if the value was added, else false.
     ********************************************************************************/
    bool Insert(Iterator& iterator, const T& value) {
        if (iterator == nullptr) {
            return false;
        } else {
            auto node2{Node::New(value)};
            if (node2 == nullptr) return false;            
            auto node1{Node::Get(iterator)->previous};
            auto node3{node1->next};

            node1->next = node2;
            node2->previous = node1;
            node2->next = node3;
            node3->previous = node2;
            size_++;
            return true;
        }       
    }

    /********************************************************************************
     * @brief Removes value at the front of referenced list.
     ********************************************************************************/
    void PopFront(void) noexcept {
        if (size_ <= 1) {
            Clear();
        } else {
            auto node1{first_};
            auto node2{node1->next};
            node2->previous = nullptr;
            
            Node::Delete(node1);
            first_ = node2;
            size_--;
        }
        return;
    }

    /********************************************************************************
     * @brief Removes value at the back of referenced list.
     ********************************************************************************/
    void PopBack(void) noexcept {
        if (size_ <= 1) {
            Clear();
        } else {
            auto node2{last_};
            auto node1{node2->previous};
            node1->next = nullptr;
            
            Node::Delete(node2);
            last_ = node1;
            size_--;
        }
        return;
    }

    
    /********************************************************************************
     * @brief Removes value at specified position in referenced list.
     *
     * @param index
     *        Reference to iterator pointing at the value to remove.
     *
     * @return
     *        True if the value was removed, else false.
     ********************************************************************************/
    bool Remove(Iterator& iterator) {
        if (iterator == nullptr) {
            return false;
        } else {
            auto node2{iterator.This()};
            auto node1{node2->previous};
            auto node3{node2->next};

            node1->next = node3;
            node3->previous = node1;
            Node::Delete(node2);
            size_--;
            return true;
        }
    }
    
    /********************************************************************************
     * @brief Class for iterating through nodes in mutable linked lists.
     ********************************************************************************/
    class Iterator {
      public:
      
        /********************************************************************************
         * @brief Default constructor, creates empty iterator.
         ********************************************************************************/
        Iterator(void) = default;

        /********************************************************************************
         * @brief Constructor, creates iterator pointing at referenced node.
         *
         * @param node
         *        Pointer to node that the iterator is set to point at.
         ********************************************************************************/
        Iterator(Node* node) : node_{node} {}

        /********************************************************************************
         * @brief Prefix increment operator, sets the iterator to point at next node.
         ********************************************************************************/
        void operator++(void) noexcept {
            node_ = node_->next;
        }

        /********************************************************************************
         * @brief Postfix increment operator, sets the iterator to point at next node.
         ********************************************************************************/
        void operator++(int) noexcept { 
            node_ = node_->next;
        }

        /********************************************************************************
         * @brief Prefix decrement operator, sets the iterator to point at previous node.
         ********************************************************************************/
        void operator--(void) noexcept {
            node_ = node_->previous;
        }

        /********************************************************************************
         * @brief Postfix decrement operator, sets the iterator to point at previous node.
         ********************************************************************************/
        void operator--(int) noexcept {
            node_ = node_->previous;
        }

        /********************************************************************************
         * @brief Addition operator, increments the iterator specified number of times.
         *
         * @param num_increments
         *        The number of times the iterator will be incremented.
         ********************************************************************************/
        void operator+= (const size_t num_increments) noexcept {
            for (size_t i{}; i < num_increments; ++i) {
                node_ = node_->next;
            }
        }

        /********************************************************************************
         * @brief Subtraction operator, decrements the iterator specified number of 
         *        times.
         *
         * @param num_increments
         *        The number of times the iterator will be decremented.
         ********************************************************************************/
        void operator-=(const size_t num_increments) noexcept {
            for (size_t i{}; i < num_increments; ++i) {
                node_ = node_->previous;
            }
        }

        /********************************************************************************
         * @brief Equality operator, checks if the iterator points at the same node as
         *        referenced other iterator.
         *
         * @param other
         *        Reference to other iterator.
         * @return 
         *        True if the iterators point at the same node, else false.
         ********************************************************************************/
        bool operator==(const Iterator& other) noexcept {
            return node_ == other.node_ ? true : false;
        }

        /********************************************************************************
         * @brief Inequality operator, checks if the iterator and referenced other
         *        iterator points at different nodes.
         *
         * @param other
         *        Reference to other iterator.
         * @return 
         *        True if the iterators point at different nodes, else false.
         ********************************************************************************/
        bool operator!=(const Iterator& other) noexcept {
            return node_ != other.node_ ? true : false;
        }

        /********************************************************************************
         * @brief Dereference operator, returns a reference to the value stored by the
         *        node the iterator is pointing at. Not
         *
         * @return 
         *        Reference to the value stored by the node the iterator is pointing at.
         ********************************************************************************/
        T& operator*(void) noexcept {
            return node_->data;
        }

        /********************************************************************************
         * @brief Returns the address of the node the iterator points at. A void pointer
         *        is returned to keep information about nodes private within the List 
         *        class.
         *
         * @return 
         *        Pointer to the node the iterator is pointing at.
         ********************************************************************************/
        void* Address(void) {
            return node_;
        }

      private:
        Node* node_{nullptr}; /* Pointer to the node the iterator is pointing at. */
    };

    /********************************************************************************
     * @brief Class for iterating through nodes in mconstant linked lists.
     ********************************************************************************/
    class ConstIterator {
      public:

        /********************************************************************************
         * @brief Constructor, creates iterator pointing at referenced node.
         *
         * @param node
         *        Pointer to node that the iterator is set to point at.
         ********************************************************************************/
        ConstIterator(const Node* node) noexcept : node_{node} {}

        /********************************************************************************
         * @brief Prefix increment operator, sets the iterator to point at next node.
         ********************************************************************************/
        void operator++(void) noexcept { 
            node_ = node_->next;
        }

        /********************************************************************************
         * @brief Postfix increment operator, sets the iterator to point at next node.
         ********************************************************************************/
        void operator++(int) noexcept { 
            node_ = node_->next;
        }

        /********************************************************************************
         * @brief Prefix decrement operator, sets the iterator to point at previous node.
         ********************************************************************************/
        void operator--(void) noexcept { 
            node_ = node_->previous;
        }

        /********************************************************************************
         * @brief Postfix decrement operator, sets the iterator to point at previous node.
         ********************************************************************************/
        void operator--(int) noexcept {
            node_ = node_->previous;
        }

        /********************************************************************************
         * @brief Addition operator, increments the iterator specified number of times.
         *
         * @param num_increments
         *        The number of times the iterator will be incremented.
         ********************************************************************************/
        void operator+=(const size_t num_incremenets) noexcept {
            for (size_t i{}; i < num_incremenets; ++i) {
                node_ = node_->next;
            }
        }

        /********************************************************************************
         * @brief Subtraction operator, decrements the iterator specified number of 
         *        times.
         *
         * @param num_increments
         *        The number of times the iterator will be decremented.
         ********************************************************************************/
        void operator-=(const size_t num_increments) noexcept {
            for (size_t i{}; i < num_increments; ++i) {
                node_ = node_->previous;
            }
        }

        /********************************************************************************
         * @brief Equality operator, checks if the iterator points at the same node as
         *        referenced other iterator.
         *
         * @param other
         *        Reference to other iterator.
         * @return 
         *        True if the iterators point at the same node, else false.
         ********************************************************************************/
        bool operator==(ConstIterator& other) const noexcept {
            return node_ == other.node_ ? true : false;
        }

        /********************************************************************************
         * @brief Inequality operator, checks if the iterator and referenced other
         *        iterator points at different nodes.
         *
         * @param other
         *        Reference to other iterator.
         * @return 
         *        True if the iterators point at different nodes, else false.
         ********************************************************************************/
        bool operator!=(ConstIterator& other) const noexcept {
            return node_ != other.node_ ? true : false;
        }

        /********************************************************************************
         * @brief Dereference operator, returns a reference to the value stored by the
         *        node the iterator is pointing at. Not
         *
         * @return 
         *        Reference to the value stored by the node the iterator is pointing at.
         ********************************************************************************/
        const T& operator*(void) const noexcept {
            return node_->data;
        }
        
        /********************************************************************************
         * @brief Returns the address of the node the iterator points at. A void pointer
         *        is returned to keep information about nodes private within the List 
         *        class.
         *
         * @return 
         *        Pointer to the node the iterator is pointing at.
         ********************************************************************************/
        const void* Address(void) const noexcept {
            return node_;
        }

      private:
        const Node* node_{nullptr}; /* Pointer to the node the iterator is pointing at. */
    };

  private:

    /********************************************************************************
     * @brief Struct for implementation of nodes in the linked list.
     ********************************************************************************/
    struct Node {
        Node* previous; /* Pointer to previous node, if any. */
        Node* next;     /* Pointer to next next, if any. */
        T data;         /* Data stored by the node. */

        /********************************************************************************
         * @brief Allocates memory for a new node and stores referenced data.
         *
         * @param data
         *        Data to store in referenced node.
         * @return
         *        A pointer to the node if the node was created, else a null pointer.
         ********************************************************************************/
        static Node* New(const T& data) noexcept {
            Node* self{detail::New<Node>()};
            if (self == nullptr) return nullptr;
            self->data = data;
            self->previous = nullptr;
            self->next = nullptr;
            return self;
        }

        /********************************************************************************
         * @brief Deletes referenced node by deallocating heap allocated memory.
         *
         * @param self
         *        Pointer to the node to delete.
         ********************************************************************************/
        static void Delete(Node* self) noexcept {
            detail::Delete<Node>(self);
        }

        /********************************************************************************
         * @brief Returns a pointer to the node referenced iterator is pointing at.
         *
         * @param iterator
         *        Reference to an arbitrary iterator.
         * @return 
         *        A pointer to the node the iterator is pointing at. 
         ********************************************************************************/
        static Node* Get(Iterator& iterator) {
            return static_cast<Node*>(iterator.Address());
        }

       /********************************************************************************
         * @brief Returns a pointer to the node referenced iterator is pointing at.
         *
         * @param iterator
         *        Reference to an arbitrary constant iterator.
         * @return 
         *        A pointer to the node the iterator is pointing at. 
         ********************************************************************************/
        static const Node* Get(ConstIterator& iterator) noexcept {
            return static_cast<Node*>(iterator.Address());
        }
    };

    Node* first_{nullptr}; /* Pointer to the first node of the list. */
    Node* last_{nullptr};  /* Pointer to the last node of the list. */
    size_t size_{};        /* The size of the list, i.e. the number of stored elements. */

    /********************************************************************************
     * @brief Copies referenced values and stored in referenced list. All previous 
     *        values are either removed or overwritten.
     *
     * @param values
     *        Referenced values to copy.
     * @return
     *        True if all values were copied, else false.
     ********************************************************************************/
    template <size_t size>
    bool Copy(const T (&values)[size]) {
        Clear();
        for (size_t i{}; i < size; ++i) {
            if (!PushBack(values[i])) {
                return false;
            }
        }
        return true;
    }

    /********************************************************************************
     * @brief Copies the content of referenced source. All previous values are
     *        either emoved or overwritten.
     *
     * @param source
     *        Reference to list whose content is copied.
     * @return
     *        True if the content of the source list was copied, else false.
     ********************************************************************************/
    bool Copy(List& source) noexcept {
        Clear();
        for (size_t i{}; i < source.Size(); ++i) {
            if (!PushBack(source[i])) { 
                return false;
            }
        }
        return true;
    }

    /********************************************************************************
     * @brief Removes all nodes stored in referenced list.
     ********************************************************************************/
    void RemoveAllNodes(void) noexcept {
        for (auto i{begin()}; i != end();) {
            auto next{Node::Get(i)->next};
            Node::Delete(Node::Get(i));
            i = next;
        }
    }
};

} /* namespace container */
} /* namespace yrgo */