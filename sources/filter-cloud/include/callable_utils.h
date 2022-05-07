#ifndef CALLABLE_UTILS_H
#define CALLABLE_UTILS_H

/********** Headers **********/

// libc++
#include <functional> // function objects support

/********** callable_utils.h **********/
/**
 * Implementation source: 
 * https://stackoverflow.com/questions/10766112/c11-i-can-go-from-multiple-args-to-tuple-but-can-i-go-from-tuple-to-multiple
*/
namespace utils {

	// implementation details, users never invoke these directly
	namespace unpack_impl {
		template <typename Callable, typename Tuple, bool Done, int Total, int... N>
		struct call_impl {
			static void call(Callable fn, Tuple&& args) {
				call_impl<Callable, Tuple, Total == 1 + sizeof...(N), Total, N..., sizeof...(N)>::call(fn, std::forward<Tuple>(args));
			}
		};

		template <typename Callable, typename Tuple, int Total, int... N>
		struct call_impl<Callable, Tuple, true, Total, N...> {
			static void call(Callable fn, Tuple&& args) {
				fn(std::get<N>(std::forward<Tuple>(args))...);
			}
		};
	} // namespace unpack_impl

	/**
	 * Invokes callable object `fn`, forwarding the unpacked values from `args` tuple
	 * as its arguments.
	 * 
	 * @param fn Callable object to be invoked.
	 * @param args Tuple-like object containing the arguments for `fn` in the order
	 *        they are supposed to be in.
	*/
	template <typename Callable, typename Tuple>
	void invoke(Callable fn, Tuple&& args) {
		typedef typename std::decay<Tuple>::type ttype;
		unpack_impl::call_impl<Callable, Tuple, 0 == std::tuple_size<ttype>::value, std::tuple_size<ttype>::value>::call(fn, std::forward<Tuple>(args));
	}

} // namespace utils

#endif // CALLABLE_UTILS_H
