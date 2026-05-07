#ifndef PISTON_LIB_DEFORMER_DATA_CACHE_H_
#define PISTON_LIB_DEFORMER_DATA_CACHE_H_

#include "framework.h"
#include "common.h"
#include "topology.h"
#include "serializable_data.h"
#include "simple_profiler.h"
#include "logging.h"

#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <functional>
#include <algorithm>
#include <variant>


namespace Piston {

class BaseCurvesDeformer;

/*
 * Factory singleton class
 */
class DeformerDataCache {
	public:
		struct KeyBase {
			struct empty_type {};
			
			std::type_index 			type_idx;
			std::vector<pxr::SdfPath> 	paths;
			size_t              		topologies_hash_sum;
			std::vector<size_t>			topology_indices;

			KeyBase(): type_idx(typeid(KeyBase::empty_type)) {};
			KeyBase(const std::type_index& _type_idx, const UsdPrimHandle& handle);
			KeyBase(const std::type_index& _type_idx, const std::vector<const UsdPrimHandle*>& handles);
		};

		struct KeyStrict: KeyBase {
			KeyStrict(): KeyBase() {};
			KeyStrict(const std::type_index& _type_idx, const UsdPrimHandle& handle): KeyBase(_type_idx, handle) {};
			KeyStrict(const std::type_index& _type_idx, const std::vector<const UsdPrimHandle*>& handles): KeyBase(_type_idx, handles) {};

			bool operator==(const KeyStrict& other) const {
				if(type_idx != other.type_idx || paths.size() != other.paths.size() || topologies_hash_sum != other.topologies_hash_sum) return false;

				if(std::is_permutation(paths.begin(), paths.end(), other.paths.begin(), other.paths.end())) {
					static const auto& cache = DeformerDataCache::getInstance();
					return cache.topologiesAreEqual(topology_indices, other.topology_indices);
				}
				
				return true;
			}
		};

		struct Key: public KeyBase {
			Key(): KeyBase() {};
			Key(const std::type_index& _type_idx, const UsdPrimHandle& handle): KeyBase(_type_idx, handle) {};
			Key(const std::type_index& _type_idx, const std::vector<const UsdPrimHandle*>& handles): KeyBase(_type_idx, handles) {};

			bool operator==(const Key& other) const {
				if(type_idx != other.type_idx || paths.size() != other.paths.size() || topologies_hash_sum != other.topologies_hash_sum) return false;

				if(!std::is_permutation(paths.begin(), paths.end(), other.paths.begin(), other.paths.end())) {
					static const auto& cache = DeformerDataCache::getInstance();
					return cache.topologiesAreEqual(topology_indices, other.topology_indices);
				}
				
				return true;
			}
		};

		using KeyVariant = std::variant<Key, KeyStrict>;

		struct KeyComparator {
			bool operator()(const Key& lhs, const Key& rhs) const {
				return lhs == rhs;
			}

			bool operator()(const KeyStrict& lhs, const KeyStrict& rhs) const {
				return lhs == rhs;
			}

			bool operator()(const KeyVariant& lhs, const KeyVariant& rhs) const {
				// If they hold different types (different indices), they aren't equal
				if (lhs.index() != rhs.index()) return false;

				// Compare the values using std::visit
				return std::visit([](auto&& l, auto&& r) -> bool {
					// Check if types are exactly the same before comparing
					if constexpr (std::is_same_v<decltype(l), decltype(r)>) {
						return l == r;
					}
					return false;
				}, lhs, rhs);
			}
		};

		struct KeyHasher {
			std::size_t operator()(const KeyVariant& k) const {
				// Use std::visit to apply the correct std::hash based on the active type
				std::size_t h = std::visit([](auto&& arg) -> std::size_t {

					auto hash_combine = [](std::size_t& s, std::size_t h) {
						s ^= h + 0x9e3779b9 + (s << 6) + (s >> 2);
					};
					
					size_t seed = 0;
					hash_combine(seed, arg.type_idx.hash_code());

					auto ordered_paths = arg.paths;
					std::sort(ordered_paths.begin(), ordered_paths.end());

					hash_combine(seed, ordered_paths.size());
					hash_combine(seed, arg.topologies_hash_sum);

					return seed;
				}, k);

				// Optionally combine with the index to distinguish between 
				// identical values in different variant slots (e.g., variant<int, int>)
				return h ^ (std::hash<size_t>{}(k.index()) << 1);
			}

			std::size_t operator()(const Piston::DeformerDataCache::KeyStrict& k) const {
				return 0;
			}

			std::size_t operator()(const Piston::DeformerDataCache::Key& k) const {
				std::size_t seed = 0;

				auto hash_combine = [](std::size_t& s, std::size_t h) {
					s ^= h + 0x9e3779b9 + (s << 6) + (s >> 2);
				};

				hash_combine(seed, k.type_idx.hash_code());

				auto ordered_paths = k.paths;
				std::sort(ordered_paths.begin(), ordered_paths.end());

				hash_combine(seed, ordered_paths.size());
				hash_combine(seed, k.topologies_hash_sum);

				return seed;
			}
		};

		class BiMap {
			std::unordered_map<Key, std::shared_ptr<SerializableDeformerDataBase>, KeyHasher, KeyComparator> mKeyToValMap;
			std::unordered_map<std::shared_ptr<SerializableDeformerDataBase>, Key> mValToKeyMap;

			public:
				void insert(const Key& k, const std::shared_ptr<SerializableDeformerDataBase>& v) {
					mKeyToValMap[k] = v;
					//mValToKeyMap[v] = k;
					mValToKeyMap.insert_or_assign(v, k);
				}

				void eraseByKey(const Key& k) {
					auto it = mKeyToValMap.find(k);
					if (it != mKeyToValMap.end()) {
			    		mValToKeyMap.erase(it->second);
			    		mKeyToValMap.erase(it);
					}
				}

				void eraseByValue(const std::shared_ptr<SerializableDeformerDataBase>& v) {
					auto it = mValToKeyMap.find(v);
					if (it != mValToKeyMap.end()) {
			    		mKeyToValMap.erase(it->second);
			    		mValToKeyMap.erase(it);
					}
				}
		};

		using MapType = std::unordered_map<KeyVariant, std::shared_ptr<SerializableDeformerDataBase>, KeyHasher, KeyComparator>;

	public:
		~DeformerDataCache();

		// Deleting the copy constructor to prevent copies
		DeformerDataCache(const DeformerDataCache& obj) = delete;

		// Static method to get the CurvesDeformerFactory instance
		static DeformerDataCache& getInstance();

		template< class T>
		std::shared_ptr<T> getOrCreateData(const BaseCurvesDeformer* pDeformer, const UsdPrimHandle& handle, bool& created);

		template< class T>
		std::shared_ptr<T> getOrCreateData(const BaseCurvesDeformer* pDeformer, const std::vector<const UsdPrimHandle*>& handles, bool& created);

		template< class T>
		void invalidate(const UsdPrimHandle& handle);

		template< class T>
		void invalidate(const std::vector<const UsdPrimHandle*>& handles);

		// Remove data from cache completely
		template< class T>
		void invalidate(const std::shared_ptr<T>& pData);

		void cleanup();

	protected:
		void clear();

	private:
		bool topologiesAreEqual(const std::vector<size_t>& indices_l, const std::vector<size_t>& indices_r) const {
			assert(indices_l.size() == indices_r.size());
			if(indices_l.size() != indices_r.size()) return false;
			
			const std::lock_guard<std::mutex> lock(mTopologyPoolMutex);

			for(size_t i = 0; i < indices_l.size(); ++i) {
        		if( mTopologyPool[indices_l[i]] != mTopologyPool[indices_r[i]]) return false;
        	}

        	return true;
		}

		size_t getTopologyIndexFromPool(const Topology& topology) {
			const std::lock_guard<std::mutex> lock(mTopologyPoolMutex);

			auto lambda = [topology](const Topology& t) { return t.topology_hash == topology.topology_hash && t.topology_variant == topology.topology_variant; };

			auto it = std::find_if(mTopologyPool.begin(), mTopologyPool.end(), lambda);

			if (it != mTopologyPool.end()) {
		        return std::distance(mTopologyPool.begin(), it);
		    } 
		    
		    mTopologyPool.push_back(topology);
		    return mTopologyPool.size() - 1;
		}

	private:
		MapType mDataMap;
		std::vector<Topology> mTopologyPool;

		// Mutex to ensure thread safety
		static std::mutex mMutex;
		static std::mutex mTopologyPoolMutex;

		// Static pointer to the CurvesDeformerFactory instance
		static DeformerDataCache* mInstancePtr;

		bool mUseDataInstancing = false;

		DeformerDataCache();
};

} // namespace Piston

#endif // PISTON_LIB_CURVES_DEFORMER_FACTORY_H_