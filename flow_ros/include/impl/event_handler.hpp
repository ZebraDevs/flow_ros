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
#include <tuple>
#include <type_traits>

// Flow
#include <flow_ros/message_ptr.h>
#include <flow_ros/publisher.h>
#include <flow_ros/subscriber.h>

// Flow
#include <flow/impl/apply.hpp>
#include <flow/impl/integer_sequence.hpp>


namespace flow_ros
{
namespace detail
{

/**
 * @brief Wraps tuple element types in with template to generate a new tuple type
 */
template <template <typename...> class WrapperTmpl, typename ChannelType> struct WrapTupleElements
{
  using type = std::tuple<WrapperTmpl<ChannelType>>;
};


/**
 * @copydoc WrapTupleElements
 * @note Multi-element specialization
 */
template <template <typename...> class WrapperTmpl, typename... ChannelTs>
struct WrapTupleElements<WrapperTmpl, std::tuple<ChannelTs...>>
{
  using type = std::tuple<WrapperTmpl<ChannelTs>...>;
};


/**
 * @brief Dereferences pointer values in a tuple and forwards those values in a new tuple
 */
template <typename PtrTupleT, std::size_t... IntPack>
decltype(auto) forward_as_deref_tuple_impl(PtrTupleT&& ptr_tuple, ::flow::index_sequence<IntPack...>)
{
  return std::forward_as_tuple(*std::get<IntPack>(ptr_tuple)...);
}


/**
 * @copydoc forward_as_deref_tuple
 */
template <typename PtrTupleT> decltype(auto) forward_as_deref_tuple(PtrTupleT&& ptr_tuple)
{
  constexpr auto N = std::tuple_size<typename std::remove_reference<PtrTupleT>::type>::value;
  return forward_as_deref_tuple_impl(std::forward<PtrTupleT>(ptr_tuple), ::flow::make_index_sequence<N>{});
}


/**
 * @brief Collects objects from a tuples into an output container via iterator
 */
template <typename OutputIteratorT> class CollectFromTuple
{
public:
  explicit CollectFromTuple(OutputIteratorT output_itr) : output_itr_{output_itr} {}

  template <typename ChannelPtrT> inline void operator()(ChannelPtrT channel) { *(output_itr_++) = channel; }

private:
  /// Output iterator
  OutputIteratorT output_itr_;
};


/**
 * @brief Helper function use to set message sequencing stamp
 *
 * @tparam MsgT  (deduced) message type
 * @tparam SeqT  (deduced) sequencing type
 *
 * @param msg  message
 * @param seq  message sequence counter
 */
template <typename MsgT, typename SeqT> inline void set_stamp(MsgT&& msg, SeqT&& seq)
{
  StampSetter<MsgT>{}(std::forward<MsgT>(msg), std::forward<SeqT>(seq));
}


/**
 * @brief Helper object used to publish 1 or N outputs from an EventHandler
 */
class EventHandlerPublishHelper
{
public:
  explicit EventHandlerPublishHelper(const ros::Time stamp) : stamp_{stamp} {}

  /// Publishes multiple messages through a publisher implementation
  template <typename PubT, typename OutputT> inline void operator()(std::shared_ptr<PubT>& pub, OutputT&& output) const
  {
    EventHandlerPublishHelper::call_publish(*pub, std::forward<OutputT>(output), stamp_);
  }

private:
  /// Publishes multiple messages through a MultiPublisher
  template <typename MsgT, typename OutputContainerT>
  static inline void
  call_publish(MultiPublisher<MsgT, OutputContainerT>& pub, OutputContainerT&& output, const ros::Time& stamp)
  {
    pub.publish(std::forward<OutputContainerT>(output));
  }

  /// Publishes message through a Publisher; automatically sets stamp to match driving message stamp
  template <typename MsgT>
  static inline std::enable_if_t<!std::is_const<MsgT>::value>
  call_publish(Publisher<MsgT>& pub, message_shared_ptr_t<MsgT> output, const ros::Time& stamp)
  {
    if (output)
    {
      set_stamp(*output, stamp);
    }
    pub.publish(std::move(output));
  }

  /// Publishes message through a Publisher; does not automatically sets stamp to match driving message stamp
  template <typename MsgT>
  static inline std::enable_if_t<std::is_const<MsgT>::value>
  call_publish(Publisher<MsgT>& pub, message_shared_ptr_t<MsgT> output, const ros::Time& stamp)
  {
    pub.publish(std::move(output));
  }

  /// Output time stamp
  const ros::Time stamp_;
};


/**
 * @brief Helper object used update queue monitor/preconditioner
 *
 *        Uses an intentionally non-code state enum, since this isn't a capture state, though
 *        we might want to reset monitoring when execution is bypassed
 */
class QueueMonitorSignallingHelper
{
public:
  template <typename CaptorT, typename LockableT, typename QueueMonitorT>
  inline void operator()(::flow::Captor<CaptorT, LockableT, QueueMonitorT>& c) const
  {
    c.update_sync_state(::flow::State::_N_STATES);
  }
};


/**
 * @brief Helper object used to re-inject messages when sync must be retried
 */
struct RetryReinjectHelper
{
  template <typename SubscriberPtrT, typename InputContainerT>
  inline void operator()(SubscriberPtrT& sub, InputContainerT& c) const
  {
    sub->insert(c.begin(), c.end());
  }
};


/**
 * @brief Generates event callback output type for an EventHandler with a single output
 */
template <typename PublisherTupleT> struct EventHandlerOutputType
{
  using type = typename PublisherTraits<PublisherTupleT>::OutputType;
};


/**
 * @brief Generates event callback output type for an EventHandler with NO outputs
 */
template <> struct EventHandlerOutputType<std::tuple<>>
{
  using type = std::tuple<>;
};


/**
 * @brief Generates event callback output type for an EventHandler with multiple outputs
 */
template <typename... PublisherTs> struct EventHandlerOutputType<std::tuple<PublisherTs...>>
{
  using type = std::tuple<typename PublisherTraits<PublisherTs>::OutputType...>;
};


/**
 * @brief Generates event callback input type for an EventHandler
 */
template <template <typename> class OutputContainerTypeInfoTmpl, typename SubscriberTupleType>
struct EventHandlerInputType;


/**
 * @brief Generates event callback input type for an EventHandler with multiple outputs
 */
template <template <typename> class OutputContainerTypeInfoTmpl, typename... SubscriberTs>
struct EventHandlerInputType<OutputContainerTypeInfoTmpl, std::tuple<SubscriberTs...>>
{
  // Does the following:
  //
  //  1. Extracts DispatchType from subscribers
  //  2. Gets a container type which holds DispatchType
  //  3. Creates a tuple of DispatchType containers for each SubscriberTs
  //
  using type = std::tuple<typename OutputContainerTypeInfoTmpl<
    typename ::flow::CaptorTraits<typename SubscriberTraits<SubscriberTs>::PolicyType>::DispatchType>::Container...>;
};


/**
 * @brief Generates a tuple of output iterators
 */
template <
  template <typename>
  class OutputContainerTypeInfoTmpl,
  typename SubscriberTupleT,
  typename ContainerTupleT,
  std::size_t... IntPack>
decltype(auto) get_ouput_iterators_impl(ContainerTupleT&& c_tuple, ::flow::index_sequence<IntPack...>)
{
  // Does the following:
  //
  //  1. Extracts DispatchType from subscribers
  //  2. Creates a tuple of DispatchType's for each SubscriberTs
  //
  using DispatchTypeTuple = std::tuple<typename ::flow::CaptorTraits<
    typename SubscriberTraits<std::tuple_element_t<IntPack, SubscriberTupleT>>::PolicyType>::DispatchType...>;

  // Create a tuple of output iterators for each DispatchType container
  return std::make_tuple(
    OutputContainerTypeInfoTmpl<std::tuple_element_t<IntPack, DispatchTypeTuple>>::get_output_iterator(
      std::get<IntPack>(c_tuple))...);
}


/**
 * @copydoc get_ouput_iterators
 */
template <template <typename> class OutputContainerTypeInfoTmpl, typename SubscriberTupleT, typename ContainerTupleT>
decltype(auto) get_ouput_iterators(ContainerTupleT&& c_tuple)
{
  constexpr auto N = std::tuple_size<typename std::remove_reference_t<ContainerTupleT>>::value;
  return get_ouput_iterators_impl<OutputContainerTypeInfoTmpl, SubscriberTupleT>(
    std::forward<ContainerTupleT>(c_tuple), ::flow::make_index_sequence<N>{});
}

}  // namespace detail
}  // namespace flow_ros

#endif  // FLOW_ROS_IMPL_EVENT_HANDLER_HPP
