/**
* Word of Notice and Disclosure:
* This module is an open source light weight yaml parser created by the original author.
* Anyone is free to contribute/use/edit the parser for all kind off stuff. However, the original author and the github link of the original author (Mohammed Ghaith Al-Mahdawi (Mohido)) and module must be clearly disclosed and mentioned.
* Contribution are welcome! but requires the original author must approve the code before merging to the master/development branches first.
*
*
*
*
* Original Author: Mohammed Ghaith Al-Mahdawi (Mohido)
* Module: Tiny Yaml parser
* Official Remote Repository: https://github.com/Mohido/Tiny_Yaml.git
* How to Use:
*	1) Create a yaml object: `TINY_YAML::Yaml coolYamlObject(<yaml_file_path>);`
*	2) The object will parse the yaml file during initialization
*	3) Then use the brackets `[]` to access the yaml data:  `coolYamlObject[<node_ID>][<node_ID>].getData<std::string>();`
*	4) In point 3, we are accessing the value of the following yaml:
*			<node_ID>:
*				<node_ID>: <value>
*
* Notes:
*	- All values are stored as strings. The developer will have to go through changing a string to number if needed. (e.g: if value is 123, it is stored as string)
*	- At the moment there is no save into file function since it is using the std::unordered_map for storing the children nodes.
*	- This is a light-weight library, meaning that it will not handle exceptions and missuse of data. For instance, accessing a node that does not exist will not be handled by the library.
*
*
* Example Yaml file:
*		|-------------------------------------|
*		| object:							  |
*		|   name: "mohido"					  |
*		|   property: "cool as hell"		  |
*		| 									  |
*		| version: 123						  |
*		| academy: "333"					  |
*		| 									  |
*		| list:								  |
* 		|   - item1							  |
* 		|   - item2							  |
*		|   - item3							  |
*		| 									  |
*		| node_list: 						  |
*		|   - name : "node1"				  |
*		|     value: 1						  |
*		|     hell:							  |
*		|       - item1: "sang"				  |
*		|       - item1: "mang"				  |
*		|   - name : "node2"				  |
*		|     value: 2						  |
*		|     temp: extra					  |
*		|     extra:						  |
*		|       - extra1					  |
*		|       - extra2					  |
*		|       - extra3					  |
*		| 									  |
*		|-------------------------------------|
*
* -	value <"mohido"> in <object.name> can be accessed as follows:
*		coolYamlObject["object"]["name"].getData<std::string>();			// Note, this returns a reference to the data itself, the user can copy it into a new object or use the object directly.
*
* - value <123> of the <version> can be accessed as follows:
*		coolYamlObject["version"].getData<std::string>();
*
* - value <item1> of <list[0]> can be accessed as follows:
*		coolYamlObject["list"].getData<std::vector<std::string>>()[0];		// When a node contains a list of elements, it can be accessed via `getData<std::vector<std::string>>`
*
* - value <"node1"> of <node_list[0].name> can be accessed as follows:
*		coolYamlObject["node_list"]["0"]["name"].getData<std::string>();	// When a node contains a list of nodes, it can be access via `["<nodes_index>"]`
*
* - value <extra3> of <node_list[1].extra[2]> can be accessed as follows:
*		coolYamlObject["node_list"]["1"]["extra"].getData<std::vector<std::string>>()[2];
*/
#pragma once

#ifndef TINY_YAML_PARSER
#define TINY_YAML_PARSER

#define MAX_CHARACTERS_IN_LINE 512


#include <memory>
#include <string>
#include <unordered_map>
#include <iostream>

namespace TINY_YAML {

	/// <summary>
	/// If a node is a leaf => it has data. Otherwise, it has only children nodes.
	/// </summary>
	class Node {
	private:
		std::string m_identifier;														// name of the node
		std::shared_ptr<void> m_data;													// Contains the data of the node
		std::unordered_map<std::string, std::shared_ptr<Node>> m_children;				// Holds data to the children nodes

	public:
		/// <summary>
		///
		/// </summary>
		/// <param name="identifier"></param>
		/// <param name="data"></param>
		/// <param name="size"></param>
		Node(const std::string& identifier, std::shared_ptr<void> data);
		~Node();

		/// <summary>
		///
		/// </summary>
		/// <param name="node"></param>
		/// <returns></returns>
		bool append(std::shared_ptr<Node> node);

		/// <summary>
		///
		/// </summary>
		/// <returns></returns>
		unsigned int getSize() {
			return static_cast<unsigned int>(this->m_children.size());
		}

		/// <summary>
		///
		/// </summary>
		/// <param name="result"></param>
		/// <returns></returns>
		std::string& getID();

		/// <summary>
		///
		/// </summary>
		/// <typeparam name="T"></typeparam>
		/// <param name=""></param>
		/// <returns></returns>
		template<typename T> T& getData() {
			return *std::static_pointer_cast<T>(m_data);
		}


		/// <summary>
		///
		/// </summary>
		friend std::ostream& operator<<(std::ostream& os, const Node& node) {
			os << node.m_identifier << " (" << &node << ")" << std::endl;
			for (const auto& it : node.m_children) {
				os << node.m_identifier << " : " << *it.second;
			}
			return os;
		}

		/// <summary>
		///
		/// </summary>
		Node& operator[](const std::string& identifier) {
			return *m_children[identifier];
		}

		const std::unordered_map<std::string, std::shared_ptr<Node>>& getChildren() const {
			return m_children;
		}

		/// <summary>
		///
		/// </summary>
		/// <param name="data"></param>
		/// <returns></returns>
		bool setData(std::shared_ptr<void> data) {
			this->m_data = data;
			return true;
		}

	};


	class Yaml {
		std::unordered_map<std::string, std::shared_ptr<Node>> m_roots;			// The root nodes in the file.
		std::string error_message;  // container for any error message
	public:
		Yaml(const std::string& filepath);
		~Yaml();

		bool load(const std::string& filepath);									// Loads data from a specific file
		// bool save(const std::string& filepath);								// Saves data too a specific file, For future release..

		friend std::ostream& operator<<(std::ostream& os, Yaml& yaml) {
			for (const auto& it: yaml.m_roots) {
				os << *it.second;
			}
			os << std::endl;
			return os;
		}

		Node& operator[](const std::string& identifier) {
			Node& node = *m_roots[identifier];
			if (std::addressof(node) == 0)
			{
				throw std::runtime_error("Failed to locate key: " + identifier);
			}
            return node;
		}

		const std::unordered_map<std::string, std::shared_ptr<Node>>& getNodes() const {
			return m_roots;
		}
	};


	template<typename F, typename S, typename T>
	struct Triple {
		F first;
		S second;
		T third;

		Triple(F first, S second, T third)
			: first(first), second(second), third(third)
		{}
	};
}



#endif
