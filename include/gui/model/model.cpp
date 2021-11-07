#include "model.h"

namespace DynaMap{

using namespace std;

	void model::loadFile(std::string fileName){
		// object of the file being read
		ifstream meshFile(fileName);	
		// std::cout << "Mesh address: "<< fileName << std::endl;

		// string to save the lines of the file that is being read
		string meshFileLine;				
	
		if (meshFile.is_open()) {
			while (! meshFile.eof() ){
				// getting lines of the file
				getline(meshFile, meshFileLine);

				// assess first letter in each line
				// if the line starts with 'v', it's declaring a vertice
				if (meshFileLine.c_str()[0] == 'v'){
					if (meshFileLine.c_str()[1] == 'n'){				
						normalsStringList.push_back(meshFileLine);						
					}else{
						float x, y, z;
						// get rid of 'v'
						meshFileLine[0] = ' ';		
						// assign the vertice's values to x,y,z				
						sscanf(meshFileLine.c_str(),"%f %f %f ", &x, &y, &z);	
						// save the values into the vector vertexList of	
						this->vertexList.push_back(x);						
						this->vertexList.push_back(y);						
						this->vertexList.push_back(z);
						this->vertexList.push_back(1.0f);
					}																	
				}
				// scanning for 'f' and adding polygons to display list
				// if the line starts with 'f', it's declaring a face/polygon
				else if (meshFileLine.c_str()[0] == 'f'){				
					facesStringList.push_back(meshFileLine);

					meshFileLine[0] = ' ';			// get rid of 'f' from the line string
					istringstream iss(meshFileLine);
					while(iss){
						//count ++;
						int value;
						iss >> value;					// get values one by one
						if (iss.fail())break;
						// if it fails to get a value, then break out of loop
						this->glModel.push_back(this->vertexList.at(4*(value-1)));
						this->glModel.push_back(this->vertexList.at(4*(value-1) + 1)); 
						this->glModel.push_back(this->vertexList.at(4*(value-1) + 2));
						this->glModel.push_back(this->vertexList.at(4*(value-1) + 3));
					}				
				}
			} 
			
			// NOTE: this will only run once, after all the vertices 
			// have been added and right before polygons/faces are added
			// this->transform();

			// go back to beginning of the file
			meshFile.clear();
			meshFile.seekg(0, std::ios::beg);
		
		}else{
			std::cerr << "Can't Open Mesh file!!" << std::endl;
		}			
	}
	void model::resetPose(void){
		this->x = 0;
		this->y = 0;
		this->z = 0;
		this->roll = 0;
		this->pitch = 0;
		this->yaw = 0;
	}
	void model::saveModel(void){
		std::ofstream saveObj;
		saveObj.open("../currentModel.obj");
		
		// go through each vertice and push it to file 'currentModel.obj'
		for (unsigned int h = 0; h < this->vertexList.size(); h += 4){
			saveObj << "v " << this->vertexList.at(h)   << " " <<
				               this->vertexList.at(h+1) << " " << 
							   this->vertexList.at(h+2) << std::endl;
		}
		
		// go through each 'vn' polygon and push it to file 'currentModel.obj'
		for (unsigned int h =0; h < this->normalsStringList.size(); h++){
			saveObj << this->normalsStringList.at(h) << std::endl;
		}

		// go through each 'f' polygon and push it to file 'currentModel.obj'
		for (unsigned int h =0; h < this->facesStringList.size(); h++){
			saveObj << this->facesStringList.at(h) << std::endl;
		}
		
	}
	void model::getVertex(void){
		this->meanX = 0;
		this->meanY = 0;
		this->meanZ = 0;

		for (int q = 0; q < (this->vertexList.size() / 3) ; q++){
			//cout << this->vertexList.at(q) << endl;
			this->meanX += this->vertexList.at(3*q);
		}

		for (int q = 0; q < (this->vertexList.size() / 3) ; q++){
			//cout << this->vertexList.at(q) << endl;
			this->meanY += this->vertexList.at(3*q + 1);
		}

		for (int q = 0; q < (this->vertexList.size() / 3) ; q++){
			//cout << this->vertexList.at(q) << endl;
			this->meanZ += this->vertexList.at(3*q+2);
		}

		this->meanX = this->meanX / (this->vertexList.size() / 3);
		this->meanY = this->meanY / (this->vertexList.size() / 3);
		this->meanZ = this->meanZ / (this->vertexList.size() / 3);
	}
	void model::getboundary(){
		this->maxX = this->vertexList.at(0);
		for (int q = 0; q < ( this->vertexList.size() / 3 ); q++){
			if (this->vertexList.at(3*q) > this->maxX){
				this->maxX = this->vertexList.at(3*q);
			}
		}

		this->minX = this->vertexList.at(0);
		for (int q = 0; q < ( this->vertexList.size() / 3 ); q++){
			if (this->vertexList.at(3*q) < this->minX){
				this->minX = this->vertexList.at(3*q);
			}
		}

		this->maxY = this->vertexList.at(1);
		for (int q = 0; q < ( this->vertexList.size() / 3 ); q++){
			if (this->vertexList.at(3*q + 1) > this->maxY){
				this->maxY = this->vertexList.at(3*q + 1);
			}
		}

		this->minY = this->vertexList.at(1);
		for (int q = 0; q < ( this->vertexList.size() / 3 ); q++){
			if (this->vertexList.at(3*q + 1) < this->minY){
				this->minY = this->vertexList.at(3*q + 1);
			}
		}

		this->maxZ = this->vertexList.at(2);
		for (int q = 0; q < ( this->vertexList.size() / 3 ); q++){
			if (this->vertexList.at(3*q + 2) > this->maxZ){
				this->maxZ = this->vertexList.at(3*q + 2);
			}
		}

		this->minZ = this->vertexList.at(2);
		for (int q = 0; q < ( this->vertexList.size() / 3 ); q++){
			if (this->vertexList.at(3*q + 2) < this->minZ){
				this->minZ = this->vertexList.at(3*q + 2);
			}
		}

		float scale_x = this->maxX - this->minX;
		float scale_y = this->maxY - this->minY;
		float scale_z = this->maxZ - this->minZ;

		this->maxScale = scale_x;					//get the max scale
		if (scale_y > this->maxScale)			//the scale is max(xmax - xmin, ymax -ymin, zmax -zmin)
			this->maxScale = scale_y;

		if (scale_z > this->maxScale)
			this->maxScale = scale_z;
	}
	void model::transform(){

		this->getVertex();
		this->getboundary();

		// translate all vertices by substracting them by their means
		// substract each x  of each vertice by mean x
		for (unsigned int h = 0; h < ( this->vertexList.size() / 3); h++){					
			this->vertexList.at(3*h) = this->vertexList.at(3*h) - this->meanX;
		}
		for (unsigned int h = 0; h < ( this->vertexList.size() / 3); h++){
			this->vertexList.at(3*h + 1) = this->vertexList.at(3*h + 1) - this->meanY;
		}
		
		for (unsigned int h = 0; h < ( this->vertexList.size() / 3); h++){
			this->vertexList.at(3*h + 2) = this->vertexList.at(3*h + 2) - this->meanZ;
		}
		//getVertex();
		// scale every coordinate in the vector object
		for (unsigned int h = 0; h < this->vertexList.size() ; h++){							
			this->vertexList.at(h) = this->vertexList.at(h) / this->maxScale;
		}   
		// this is a translation of (0,0,-2)
		for (unsigned int h = 0; h < (this->vertexList.size() / 3); h++){		
			// equivalent to glTranslatef(0,0,-2) but applied			
			this->vertexList.at(3*h + 2) -= 2;									
		}	
	}
	float* model::getArray(void){
		float* arr = new float[this->vertexList.size()];
    	std::copy(this->vertexList.begin(), this->vertexList.end(), arr);
		return arr;
	}
	
} // namespace DynaMap
