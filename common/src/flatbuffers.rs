/// Iterates over a `flatbuffers::Vector`.
pub fn vector_iter<'a, T: flatbuffers::Follow<'a>>(
    xs: flatbuffers::Vector<'a, flatbuffers::ForwardsUOffset<T>>,
) -> impl Iterator<Item = <T as flatbuffers::Follow<'_>>::Inner> {
    (0..xs.len()).map(move |i| xs.get(i))
}
