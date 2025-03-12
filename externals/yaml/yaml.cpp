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
#include "yaml.hpp"

#include <vector>
#include <tuple>
#include <string>
#include <unordered_map>
#include <memory>
#include <stack>
#include <fstream>

namespace TINY_YAML {


	/////////////////////////////// NODE CLASS METHODS ///////////////////////////////
	Node::Node(const std::string& identifier, std::shared_ptr<void> data)
	: m_identifier(identifier), m_data(data)
	{}


	Node::~Node() {
		m_children.clear();
	}


	std::string& Node::getID() {
		return this->m_identifier;
	}


	bool Node::append(std::shared_ptr<Node> node) {
		std::string& nid = node->getID();
		if (this->m_children.find(nid) != this->m_children.end()) {
			return false;
		}
		this->m_children.insert({ nid, node });
		return true;
	}


	/////////////////////////////// YAML CLASS METHODS ///////////////////////////////
	Yaml::Yaml(const std::string& filepath) {
        if (!load(filepath))
        {
            error_message += "ERROR: Failed parsing " + filepath;
			throw std::runtime_error(error_message.c_str());
		}
	}


	Yaml::~Yaml() {
		this->m_roots.clear();
	}

	bool Yaml::load(const std::string& filepath) {
		/*Variables*/
		this->m_roots.clear();
		std::ifstream file(filepath, std::ios_base::in);
		std::stack<Triple<std::shared_ptr<Node>, unsigned int, bool>> parentsStack;		// Holds the parents stack pointers, their indentation and if they contain list values.
		char buf[MAX_CHARACTERS_IN_LINE];

		/*Check the yaml file*/
		if (!file.is_open()) {
			error_message += filepath + " cannot be opened";
			file.close();
			return false;
		}

		/*Read the yaml file line by line*/
		unsigned int line = 0;
		bool faulty = false;
		while (!file.eof() && !faulty) {
			file.getline(buf, MAX_CHARACTERS_IN_LINE);
			line++;
			if (buf[0] == '\0')
				continue;

			/*Get the positions of the yaml textmarks*/
			std::string lineContent = buf;		// Transform it into a string to use C++ string methods

			std::size_t hashPos = std::string::npos;
			std::size_t fstQuotePos = std::string::npos;
			std::size_t lstQuotePos = std::string::npos;
			std::size_t dashPos = std::string::npos;
			// Find special characters which takes NO affect if they are in "" or ''
			for (size_t i = 0; i < lineContent.length(); i++) {
				char c = lineContent[i];
				switch (c)
				{
				case '-': {
					// DO NOT update If ' or " then comes and not ended. And if it is already assigned
					if( !(fstQuotePos != std::string::npos && lstQuotePos == std::string::npos) && dashPos == std::string::npos) {
						dashPos = i;
					}}
					break;
				case '#' : {
					// DO NOT update If ' or " comes and not ended. And if # is already found
					if( hashPos == std::string::npos && !(fstQuotePos != std::string::npos && lstQuotePos == std::string::npos)) {
						hashPos = i;
					}}
					break;
				case '\'':
					if(hashPos == std::string::npos && lineContent[i-1] != '\\'){
						if(fstQuotePos == std::string::npos)
							fstQuotePos = i;
						else if(lstQuotePos == std::string::npos && lineContent[fstQuotePos] == c)
							lstQuotePos = i;
					}
					break;
				case '\"':
					if(hashPos == std::string::npos && lineContent[i-1] != '\\'){
						if(fstQuotePos == std::string::npos)
							fstQuotePos = i;
						else if(lstQuotePos == std::string::npos && lineContent[fstQuotePos] == c)
							lstQuotePos = i;
					}
					break;
				default:
					break;
				}
			}

			if(fstQuotePos != std::string::npos && lstQuotePos == std::string::npos){
                            error_message += "ERROR: unclosed quote found at line " + std::to_string(line) +
                                             ". Please close the quote and reparse.";
				faulty = true; break;
			}

			if (hashPos != std::string::npos)
				lineContent.erase(hashPos);

			std::size_t colonPos = lineContent.find(':');
			std::size_t firstCharPos = lineContent.find_first_not_of(" -#\t\f\v\n\r");
			std::size_t lastCharPos = lineContent.find_last_not_of(" #\t\f\v\n\r");

			/*Validation layers*/
			if (firstCharPos == std::string::npos)		// If line is empty (Only white spaces), read next line
				continue;

			if (colonPos == dashPos && dashPos == std::string::npos) { // No dash and no colon in the line => Invalid
				faulty = true; break;
			}

			/*Starting building the pnode*/
			std::shared_ptr<Node> pnode;
			std::size_t nodeLastCharPos = (colonPos <= lastCharPos) ? colonPos : lastCharPos+1;
			std::string nodeID = lineContent.substr(firstCharPos, nodeLastCharPos - firstCharPos);				// Can be the pnode id or the array values.

			/* Layer up. (Current line has less indentation than the previous parent = does not belong to it)*/
			while (parentsStack.size() != 0 && parentsStack.top().second >= firstCharPos) {
				if (parentsStack.top().first->getChildren().size() == 0) {
					error_message += "Parent node (layer up) \"" + parentsStack.top().first->getID() + "\" missing children \n";
					faulty = true;
					break;
				}
				parentsStack.pop();
			}

			if (faulty) {
				break;
			}

			/* List of nodes/items */
			if (dashPos < firstCharPos) {  // a dash in the beginning indicates a list item

				parentsStack.top().third = true;		// The current parrent is found to have list items

				/*If dash comes with colon => we create a virtual node that has internal nodes */
				if (colonPos != std::string::npos) {

					/* Since the virtual nodes indentation = dashpos, we have to consider the dashpos now*/
					while (parentsStack.size() != 0 && parentsStack.top().second >= dashPos) {
                        if (parentsStack.top().first->getChildren().size() == 0) {
                            error_message += "Parent node (virtual nodes) \"" + parentsStack.top().first->getID() + "\" missing children \n";
                            faulty = true;
							break;
                        }
						parentsStack.pop();
					}

					if (faulty) {
						break;
					}

					/*Create the virtual pnode*/
					std::string id = std::to_string(parentsStack.top().first->getSize());		// Create the virtual node ID
					pnode = std::make_shared<Node>(Node(id, nullptr));							// Create a virtual pnode

					/*Append the node to the current parent*/
					if (!parentsStack.top().first->append(pnode)) {
						faulty = true;
						break;
					}
					/*Make the current node the new parent*/
					parentsStack.push(Triple<std::shared_ptr<Node>, unsigned int, bool>(pnode, static_cast<unsigned int>(dashPos), true));
					dashPos = std::string::npos;
				}
				else { /* A list of elements inside the current parent pnode */
					std::vector<std::string>* pvect = &parentsStack.top().first.get()->getData<std::vector<std::string>>();		// Get the current data
					if (pvect == nullptr) {		// Create teh data if it is null
						std::shared_ptr<void> temp = std::make_shared<std::vector<std::string>>();
						parentsStack.top().first.get()->setData(temp);
					}
					pvect = &parentsStack.top().first.get()->getData<std::vector<std::string>>();
					pvect->push_back(nodeID);
					continue;
				}
			}

			/*If the current node is a parent node with children nodes*/
			if (colonPos == lastCharPos && colonPos != std::string::npos) {
				pnode = std::make_shared<Node>(Node(nodeID, nullptr));

				if (parentsStack.size() == 0 && this->m_roots.find(nodeID) == this->m_roots.end()){ // If the node is at root level, we add it to the root
					this->m_roots.insert({ nodeID, pnode });
				}
				else if(parentsStack.size() == 0 && this->m_roots.find(nodeID) != this->m_roots.end()) { // If it is at root level and it exists already, we return false
					faulty = true; break;
				}
				else if (parentsStack.size() != 0 && !parentsStack.top().first->append(pnode)) { // If it is not at root level and it is failed to attach the current node to the current parent
					faulty = true; break;
				}
				parentsStack.push(Triple<std::shared_ptr<Node>, unsigned int, bool>(pnode, static_cast<unsigned int>(firstCharPos), false));
				continue;
			}

			/*Single Node containing a value"*/
			if (colonPos < lastCharPos && lastCharPos != std::string::npos ) {
				/*value extraction*/
				std::string value = lineContent.substr(colonPos + 1, lastCharPos - colonPos); // value extraction.
				value.erase(0, value.find_first_not_of(" \t\f\v\n\r"));

				/*Build pnode*/
				pnode = std::make_shared<Node>(Node(nodeID, std::make_shared<std::string>(value)));

				if (parentsStack.size() == 0) {	// Insert at root level
					if (this->m_roots.find(nodeID) == this->m_roots.end()) {
						this->m_roots.insert({ nodeID, pnode });
					}
					else
					{
						std::cerr << "Duplicate (root) key \"" << value << "\" found at line: " << line << std::endl;
						faulty = true;
						break;
					}
                }
                else if (!parentsStack.top().first->append(pnode)) { // Insert at parent level
					std::cerr << "Duplicate (parent) key \"" << value << "\" found at line: " << line << std::endl;
                    faulty = true;
                    break;
                }
			}
		}

		// check for empty parents
		while (faulty == false && parentsStack.size() != 0) {
			if (parentsStack.top().first->getChildren().size() == 0 && !parentsStack.top().third) {
				error_message += "Parent node (final check) \"" + parentsStack.top().first->getID() + "\" missing children\n";
				faulty = true;
				break;
			}
			parentsStack.pop();
		}

		if (faulty) {
			error_message += "Yaml Parser: Error: Failed to parse file, invalid yaml syntax at line: " + std::to_string(line) + "\n";
			file.close();
			this->m_roots.clear();
			return false;
		}

		file.close();
		return true;
	}

	// For future release if needed.
	/*bool Yaml::save(const std::string& filepath) {
		return true;
	}*/



}




