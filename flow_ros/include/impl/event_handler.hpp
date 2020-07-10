/**
 * @copyright 2020 Fetch Robotics Inc. All rights reserved
 * @author Brian Cairl
 *
 * @warning IMPLEMENTATION ONLY: THIS FILE SHOULD NEVER BE INCLUDED DIRECTLY!
 */
#ifndef FLOW_ROS_IMPL_EVENT_HANDLER_HPP
#define FLOW_ROS_IMPL_EVENT_HANDLER_HPP

// C++ Standard Library
#include <functional>
#include <type_traits>
#include <tuple>

// Flow
#include <flow_ros/publisher.h>
#include <flow_ros/subscriber.h>
#include <flow_ros/messages.h>

// Flow
#include <flow/impl/integer_sequence.hpp>


namespace flow_ros
{
namespace detail
{

/**
 * @brief Wraps tuple element types in with template to generate a new tuple type
 */
template<template<typename...> class WrapperTmpl, typename ChannelType>
struct WrapTupleElements
{
  using type = std::tuple<WrapperTmpl<ChannelType>>;
};


/**
 * @copydoc WrapTupleElements
 * @note Multi-element specialization
 */
template<template<typename...> class WrapperTmpl, typename... ChannelTs>
struct WrapTupleElements<WrapperTmpl, std::tuple<ChannelTs...>>
{
  using type = std::tuple<WrapperTmpl<ChannelTs>...>;
};


/**
 * @brief Dereferences pointer values in a tuple and forwards those values in a new tuple
 */
template<typename PtrTupleT, std::size_t... IntPack>
decltype(auto) forward_as_deref_tuple_impl(PtrTupleT&& ptr_tuple, ::flow::index_sequence<IntPack...>)
{
  return std::forward_as_tuple(*std::get<IntPack>(ptr_tuple)...);
}


/**
 * @copydoc forward_as_deref_tuple
 */
template<typename PtrTupleT>
decltype(auto) forward_as_deref_tuple(PtrTupleT&& ptr_tuple)
{
  constexpr auto N = std::tuple_size<typename std::remove_reference<PtrTupleT>::type>::value;
  return forward_as_deref_tuple_impl(std::forward<PtrTupleT>(ptr_tuple), ::flow::make_index_sequence<N>{});
}


/**
 * @brief Collects objects from a tuples into an output container via iterator
 */
template<typename OutputIteratorT>
class CollectFromTuple
{
public:
  explicit CollectFromTuple(OutputIteratorT output_itr) :
    output_itr_{output_itr}
  {}

  template<typename ChannelPtrT>
  inline void operator()(ChannelPtrT channel)
  {
    *(output_itr_++) = channel;
  }

private:
  /// Output iterator
  OutputIteratorT output_itr_;
};


/**
 * @brief Publishes an output
 */
template<typename OutputT>
struct OutputPublishHelper;


/**
 * @brief Specialization which publishes shared_ptr-wrapped outputs
 */
template<typename OutputT, template<typename> class SharedPtrTmpl>
struct OutputPublishHelper<SharedPtrTmpl<OutputT>>
{
  template<typename PubT, bool IS_CONST = std::is_const<OutputT>::value>
  inline std::enable_if_t<!IS_CONST> operator()(PubT& pub, SharedPtrTmpl<OutputT> output, const ros::Time& stamp) const
  {
    if (output)
    {
      set_stamp(*output, stamp);
    }
    pub.publish(std::move(output));
  }

  template<typename PubT, bool IS_CONST = std::is_const<OutputT>::value>
  inline std::enable_if_t<IS_CONST> operator()(PubT& pub, SharedPtrTmpl<OutputT> output, const ros::Time& stamp) const
  {
    pub.publish(std::move(output));
  }
};


/**
 * @brief Specialization which publishes mulitple shared_ptr-wrapped outputs
 */
template<typename OutputT, template<typename> class SharedPtrTmpl>
struct OutputPublishHelper<std::vector<SharedPtrTmpl<OutputT>>>
{
  template<typename PubT, bool IS_CONST = std::is_const<OutputT>::value>
  inline std::enable_if_t<!IS_CONST> operator()(PubT& pub, std::vector<SharedPtrTmpl<OutputT>> output, const ros::Time& stamp) const
  {
    for (auto& msg_ptr : output)
    {
      if (msg_ptr)
      {
        set_stamp(*msg_ptr, stamp);
      }
    }
    pub.publish(std::move(output));
  }

  template<typename PubT, bool IS_CONST = std::is_const<OutputT>::value>
  inline std::enable_if_t<IS_CONST> operator()(PubT& pub, std::vector<SharedPtrTmpl<OutputT>> output, const ros::Time& stamp) const
  {
    pub.publish(std::move(output));
  }
};


/**
 * @brief Helper object used to publish 1 or N outputs from an EventHandler
 */
class EventHandlerPublishHelper
{
public:
  explicit EventHandlerPublishHelper(const ros::Time stamp) :
    stamp_{stamp}
  {}

  template<typename PubPtrT, typename OutputT>
  inline void operator()(PubPtrT& pub, OutputT output) const
  {
    OutputPublishHelper<std::remove_reference_t<OutputT>>{}(*pub, std::move(output), stamp_);
  }

private:
  /// Output time stamp
  const ros::Time stamp_;
};


/**
 * @brief Helper object used to re-inject messages when sync must be retried
 */
struct RetryReinjectHelper
{
  template<typename SubscriberPtrT, typename InputContainerT>
  inline void operator()(SubscriberPtrT& sub, InputContainerT& c) const
  {
    sub->insert(c.begin(), c.end());
  }
};


/**
 * @brief Generates event callback output type for an EventHandler with a single output
 */
template<typename PublisherTupleT>
struct EventHandlerOutputType
{
  using type = typename PublisherTraits<PublisherTupleT>::OutputType;
};


/**
 * @brief Generates event callback output type for an EventHandler with NO outputs
 */
template<>
struct EventHandlerOutputType<std::tuple<>>
{
  using type = std::tuple<>;
};


/**
 * @brief Generates event callback output type for an EventHandler with multiple outputs
 */
template<typename... PublisherTs>
struct EventHandlerOutputType<std::tuple<PublisherTs...>>
{
  using type = std::tuple<typename PublisherTraits<PublisherTs>::OutputType...>;
};


/**
 * @brief Generates event callback input type for an EventHandler
 */
template<template<typename> class OutputContainerTypeInfoTmpl, typename SubscriberTupleType>
struct EventHandlerInputType;


/**
 * @brief Generates event callback input type for an EventHandler with multiple outputs
 */
template<template<typename> class OutputContainerTypeInfoTmpl, typename... SubscriberTs>
struct EventHandlerInputType<OutputContainerTypeInfoTmpl, std::tuple<SubscriberTs...>>
{
  // Does the following:
  //
  //  1. Extracts DispatchType from subscribers
  //  2. Gets a container type which holds DispatchType
  //  3. Creates a tuple of DispatchType containers for each SubscriberTs
  //
  using type = std::tuple<
    typename OutputContainerTypeInfoTmpl<
      typename ::flow::CaptorTraits<
        typename SubscriberTraits<SubscriberTs>::PolicyType
      >::DispatchType
    >::Container...
  >;
};


/**
 * @brief Generates a tuple of output iterators
 */
template<template<typename> class OutputContainerTypeInfoTmpl, typename SubscriberTupleT, typename ContainerTupleT, std::size_t... IntPack>
decltype(auto) get_ouput_iterators_impl(ContainerTupleT&& c_tuple, ::flow::index_sequence<IntPack...>)
{
  // Does the following:
  //
  //  1. Extracts DispatchType from subscribers
  //  2. Creates a tuple of DispatchType's for each SubscriberTs
  //
  using DispatchTypeTuple = std::tuple<
    typename ::flow::CaptorTraits<
      typename SubscriberTraits<std::tuple_element_t<IntPack, SubscriberTupleT>>::PolicyType
    >::DispatchType...
  >;

  // Create a tuple of output iterators for each DispatchType container
  return std::make_tuple(OutputContainerTypeInfoTmpl<std::tuple_element_t<IntPack, DispatchTypeTuple>>::get_output_iterator(std::get<IntPack>(c_tuple))...);
}


/**
 * @copydoc get_ouput_iterators
 */
template<template<typename> class OutputContainerTypeInfoTmpl, typename SubscriberTupleT, typename ContainerTupleT>
decltype(auto) get_ouput_iterators(ContainerTupleT&& c_tuple)
{
  constexpr auto N = std::tuple_size<typename std::remove_reference_t<ContainerTupleT>>::value;
  return get_ouput_iterators_impl<OutputContainerTypeInfoTmpl, SubscriberTupleT>(std::forward<ContainerTupleT>(c_tuple), ::flow::make_index_sequence<N>{});
}

}  // namespace detail
}  // namespace flow_ros

#endif  // FLOW_ROS_IMPL_EVENT_HANDLER_HPP
