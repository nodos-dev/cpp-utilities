#pragma once

#include <unordered_map>
#include <vector>
#include <memory>

namespace nos
{

template <typename K>
struct PrefixTreeNode {
	std::unordered_map<K, std::unique_ptr<PrefixTreeNode<K>>> Children;
};

template <typename K>
class PrefixTree {
public:
	PrefixTree() : Root(std::make_unique<PrefixTreeNode<K>>()) {}

	void Insert(const std::vector<K>& key)
	{
		PrefixTreeNode<K>* currentNode = Root.get();
		for (const K& part : key) {
			if (currentNode->Children.find(part) == currentNode->Children.end()) {
				currentNode->Children[part] = std::make_unique<PrefixTreeNode<K>>();
			}
			currentNode = currentNode->Children[part].get();
		}
	}

	bool Contains(const std::vector<K>& key) const
	{
		const PrefixTreeNode<K>* currentNode = Root.get();
		for (const K& part : key) {
			auto it = currentNode->Children.find(part);
			if (it == currentNode->Children.end()) {
				return false;
			}
			currentNode = it->second.get();
		}
		return true;
	}

	void Search(const std::vector<K>& prefix, std::vector<std::vector<K>>& results) const
	{
		const PrefixTreeNode<K>* currentNode = Root.get();
		for (const K& part : prefix) {
			auto it = currentNode->Children.find(part);
			if (it == currentNode->Children.end()) {
				return;
			}
			currentNode = it->second.get();
		}

		std::vector<K> currentPath = prefix;
		DFS(currentNode, currentPath, results);
	}

private:
	void DFS(const PrefixTreeNode<K>* node, std::vector<K>& path, std::vector<std::vector<K>>& results) const
	{
		if (node->Children.empty()) {
			results.push_back(path);
		}

		for (const auto& [key, child] : node->Children) {
			path.push_back(key);
			DFS(child.get(), path, results);
			path.pop_back();
		}
	}

	std::unique_ptr<PrefixTreeNode<K>> Root;
};

}