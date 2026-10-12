//! Errors that display as their stable machine-readable code.

/// Implements `Display` as the stable code and `core::error::Error` for
/// error types with a `const fn code(self) -> &'static str`.
#[doc(hidden)]
#[macro_export]
macro_rules! coded_error {
    ($($error:ty),+ $(,)?) => {$(
        impl ::core::fmt::Display for $error {
            fn fmt(&self, f: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
                f.write_str(self.code())
            }
        }

        impl ::core::error::Error for $error {}
    )+};
}
