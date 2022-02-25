#pragma once

#include <vector>
#include "CommonMini.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

class OutlineCorner {
   public:
	// virtual void GetPos(double& x, double& y, double& z) = 0;
	virtual double GetHeight() = 0;
	virtual ~OutlineCorner() {}
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	virtual void Save(pugi::xml_node&){};

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;
};
class Outline {
   public:
	typedef enum {
		FILL_TYPE_GRASS,
		FILL_TYPE_CONCRETE,
		FILL_TYPE_COBBLE,
		FILL_TYPE_ASPHALT,
		FILL_TYPE_PAVEMENT,
		FILL_TYPE_GRAVEL,
		FILL_TYPE_SOIL,
		FILL_TYPE_UNDEFINED
	} FillType;

	int id_;
	FillType fillType_;
	bool closed_;
	std::vector<std::shared_ptr<OutlineCorner>> corner_;
	std::vector<std::shared_ptr<UserData>> user_data_;

	Outline(int id, FillType fillType, bool closed) : id_(id), fillType_(fillType), closed_(closed) {}

	~Outline() {}

	void AddCorner(std::shared_ptr<OutlineCorner> outlineCorner) { corner_.push_back(outlineCorner); }
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);
};

class OutlineCornerRoad : public OutlineCorner {
   public:
	OutlineCornerRoad(int roadId, double s, double t, double dz, double height);
	OutlineCornerRoad(int id, int roadId, double s, double t, double dz, double height);
	void GetPos(double& x, double& y, double& z);
	double GetHeight() { return height_; }
	void Save(pugi::xml_node&) override;

   private:
	int id_, roadId_;
	double s_, t_, dz_, height_;
};

class OutlineCornerLocal : public OutlineCorner {
   public:
	OutlineCornerLocal(int roadId,
					   double s,
					   double t,
					   double u,
					   double v,
					   double zLocal,
					   double height,
					   double heading);
	OutlineCornerLocal(int id,
					   int roadId,
					   double s,
					   double t,
					   double u,
					   double v,
					   double zLocal,
					   double height,
					   double heading);
	void GetPos(double& x, double& y, double& z);
	double GetHeight() { return height_; }
	void Save(pugi::xml_node&) override;

   private:
	int id_, roadId_;
	double s_, t_, u_, v_, zLocal_, height_, heading_;
};
