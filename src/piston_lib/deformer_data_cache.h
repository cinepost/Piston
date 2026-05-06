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


namespace Piston {

class BaseCurvesDeformer;

/*
 * Factory singleton class
 */
class DeformerDataCache {
	public:
		struct Key {
			std::type_index 			type_idx;
			std::vector<pxr::SdfPath> 	ordered_paths;
			size_t              		topologies_hash_sum;
			std::vector<size_t>			topology_indices;

/*
			bool operator<(const Key& other) const {
				if (type_idx != other.type_idx) return type_idx < other.type_idx;

				if (path == other.path) return false;

				if (topology_hash_sum != other.topology_hash_sum) return topology_hash_sum < other.topology_hash_sum;

				if (topology_variant == other.topology_variant) return false; 

				if(topology_variant < other.topology_variant) return true;

				return path < other.path;
			}
*/

			bool operator==(const Key& other) const {
				if(type_idx != other.type_idx || ordered_paths.size() != other.ordered_paths.size() || topologies_hash_sum != other.topologies_hash_sum) return false;

				if(ordered_paths != other.ordered_paths) {
					static const auto& cache = DeformerDataCache::getInstance();
					return cache.topologiesAreEqual(topology_indices, other.topology_indices);
				}
				
				return true;
			}

			Key(const std::type_index& _type_idx, const UsdPrimHandle& handle);
			Key(const std::type_index& _type_idx, const std::vector<const UsdPrimHandle*>& handles);
		};

		struct SimpleKey {
			std::type_index 			type_idx;
			std::vector<pxr::SdfPath> 	paths;
		};

		struct KeyComparator {
			bool operator()(const Key& lhs, const Key& rhs) const {
				printf("$\n");
				if(lhs.type_idx != rhs.type_idx || lhs.ordered_paths.size() != rhs.ordered_paths.size() || lhs.topologies_hash_sum != rhs.topologies_hash_sum) return false;

				if(lhs.ordered_paths != rhs.ordered_paths) {
					static const auto& cache = DeformerDataCache::getInstance();
					return cache.topologiesAreEqual(lhs.topology_indices, rhs.topology_indices);
				}
				
				return true;
			}
		};

		struct KeyHasher {
			std::size_t operator()(const Piston::DeformerDataCache::Key& k) const {
				std::size_t seed = 0;

				auto hash_combine = [](std::size_t& s, std::size_t h) {
					s ^= h + 0x9e3779b9 + (s << 6) + (s >> 2);
				};

				hash_combine(seed, k.type_idx.hash_code());
				hash_combine(seed, k.ordered_paths.size());
				hash_combine(seed, k.topologies_hash_sum);
				/*
				static const auto& pool = DeformerDataCache::getInstance().mTopologyPool;
				hash_combine(seed, k.topologies_hash_sum);

				size_t topologies_hash_sum = 0;
				for(size_t i: k.topology_indices) {
					assert(i < pool.size());
					topologies_hash_sum += pool[i].topology_hash;
				}
				hash_combine(seed, topologies_hash_sum);
				*/
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

		using MapType = std::unordered_map<Key, std::shared_ptr<SerializableDeformerDataBase>, KeyHasher, KeyComparator>;

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

		template< class T>
		void invalidate(const std::shared_ptr<T>& pData);

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