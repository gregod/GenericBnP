use std::collections::HashMap as OriginalHashMap;
use std::collections::HashSet as OriginalHashSet;
use std::hash::BuildHasherDefault;
use twox_hash::XxHash64;
use nohash_hasher::IntMap;

/// Arbitrary HashMap using more performant hashing algorithm
pub type FullHashMap<K, V> = OriginalHashMap<K, V, BuildHasherDefault<XxHash64>>;

/// Hashset using more performant hashing algorithm
pub type HashSet<K> = OriginalHashSet<K, BuildHasherDefault<XxHash64>>;

/// HashMap for Int Types using more performant hashing algorithm
pub type HashMap<K,V> = IntMap<K,V>;