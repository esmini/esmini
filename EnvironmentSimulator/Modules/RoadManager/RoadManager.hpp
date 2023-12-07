/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#ifndef OPENDRIVE_HH_
#define OPENDRIVE_HH_

#include <cmath>
#include <string>
#include <map>
#include <vector>
#include <list>
#include "pugixml.hpp"
#include "CommonMini.hpp"

#define PARAMPOLY3_STEPS 100

namespace roadmanager
{
    int GetNewGlobalLaneId();
    int GetNewGlobalLaneBoundaryId();

    /**
            Add offset to a laneId to find a relative landId considering reference lane 0
            @param lane_id Start at this lane id
            @param offset Move this number of lanes, direction based on road t-axis
            @return target lane id
    */
    int GetRelativeLaneId(int lane_id, int offset);

    class Polynomial
    {
    public:
        Polynomial() : a_(0), b_(0), c_(0), d_(0), p_scale_(1.0)
        {
        }
        Polynomial(double a, double b, double c, double d, double p_scale = 1) : a_(a), b_(b), c_(c), d_(d), p_scale_(p_scale)
        {
        }
        void Set(double a, double b, double c, double d, double p_scale = 1);
        void SetA(double a)
        {
            a_ = a;
        }
        void SetB(double b)
        {
            b_ = b;
        }
        void SetC(double c)
        {
            c_ = c;
        }
        void SetD(double d)
        {
            d_ = d;
        }
        double GetA() const
        {
            return a_;
        }
        double GetB() const
        {
            return b_;
        }
        double GetC() const
        {
            return c_;
        }
        double GetD() const
        {
            return d_;
        }
        double GetPscale() const
        {
            return p_scale_;
        }
        double Evaluate(double s) const;
        double EvaluatePrim(double s) const;
        double EvaluatePrimPrim(double s) const;

    private:
        double a_;
        double b_;
        double c_;
        double d_;
        double p_scale_;
    };

    typedef struct
    {
        double s;
        double x;
        double y;
        double z;
        double h;
    } PointStruct;

    class OSIPoints
    {
    public:
        OSIPoints()
        {
        }
        OSIPoints(std::vector<PointStruct> points) : point_(points)
        {
        }
        void Set(std::vector<PointStruct> points)
        {
            point_ = points;
        }
        std::vector<PointStruct> &GetPoints()
        {
            return point_;
        }
        PointStruct &GetPoint(int i);
        double       GetXfromIdx(int i) const;
        double       GetYfromIdx(int i) const;
        double       GetZfromIdx(int i) const;
        int          GetNumOfOSIPoints() const;
        double       GetLength() const;

    private:
        std::vector<PointStruct> point_;
    };
    /**
            function that checks if two sets of osi points has the same start/end
            @return the number of points that are within tolerance (0,1 or 2)
    */
    int CheckOverlapingOSIPoints(OSIPoints *first_set, OSIPoints *second_set, double tolerance);

    class Geometry
    {
    public:
        typedef enum
        {
            GEOMETRY_TYPE_UNKNOWN,
            GEOMETRY_TYPE_LINE,
            GEOMETRY_TYPE_ARC,
            GEOMETRY_TYPE_SPIRAL,
            GEOMETRY_TYPE_POLY3,
            GEOMETRY_TYPE_PARAM_POLY3,
        } GeometryType;

        Geometry() : s_(0.0), x_(0.0), y_(0), hdg_(0), length_(0), type_(GeometryType::GEOMETRY_TYPE_UNKNOWN)
        {
        }
        Geometry(double s, double x, double y, double hdg, double length, GeometryType type)
            : s_(s),
              x_(x),
              y_(y),
              hdg_(hdg),
              length_(length),
              type_(type)
        {
        }
        virtual ~Geometry()
        {
        }

        GeometryType GetType() const
        {
            return type_;
        }
        double GetLength() const
        {
            return length_;
        }
        virtual double GetX() const
        {
            return x_;
        }
        void SetX(double x)
        {
            x_ = x;
        }
        virtual double GetY() const
        {
            return y_;
        }
        void SetY(double y)
        {
            y_ = y;
        }
        virtual double GetHdg() const
        {
            return GetAngleInInterval2PI(hdg_);
        }
        void SetHdg(double hdg)
        {
            hdg_ = hdg;
        }
        double GetS() const
        {
            return s_;
        }
        virtual double EvaluateCurvatureDS(double ds) const = 0;
        virtual void   Print() const;
        virtual void   EvaluateDS(double ds, double *x, double *y, double *h) const;

    protected:
        double       s_;
        double       x_;
        double       y_;
        double       hdg_;
        double       length_;
        GeometryType type_;
    };

    class Line : public Geometry
    {
    public:
        Line()
        {
        }
        Line(double s, double x, double y, double hdg, double length) : Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_LINE)
        {
        }
        ~Line(){};

        void   Print() const;
        void   EvaluateDS(double ds, double *x, double *y, double *h) const;
        double EvaluateCurvatureDS(double ds) const
        {
            (void)ds;
            return 0;
        }
    };

    class Arc : public Geometry
    {
    public:
        Arc() : curvature_(0.0)
        {
        }
        Arc(double s, double x, double y, double hdg, double length, double curvature)
            : Geometry(s, x, y, hdg, length, GEOMETRY_TYPE_ARC),
              curvature_(curvature)
        {
        }
        ~Arc()
        {
        }

        double EvaluateCurvatureDS(double ds) const
        {
            (void)ds;
            return curvature_;
        }
        double GetRadius() const;
        double GetCurvature() const
        {
            return curvature_;
        }
        void Print() const;
        void EvaluateDS(double ds, double *x, double *y, double *h) const;

    private:
        double curvature_;
    };

    class Spiral : public Geometry
    {
    public:
        enum ClothoidType
        {
            CLOTHOID,
            ARC,
            LINE
        };

        Spiral() : curv_start_(0.0), curv_end_(0.0), c_dot_(0.0), x0_(0.0), y0_(0.0), h0_(0.0), s0_(0.0), clothoid_type_(CLOTHOID)
        {
        }
        Spiral(double s, double x, double y, double hdg, double length, double curv_start, double curv_end);

        double GetCurvStart() const
        {
            return curv_start_;
        }
        double GetCurvEnd() const
        {
            return curv_end_;
        }
        double GetX0() const
        {
            return x0_;
        }
        double GetY0() const
        {
            return y0_;
        }
        double GetH0() const
        {
            return h0_;
        }
        double GetS0() const
        {
            return s0_;
        }
        double GetCDot() const
        {
            return c_dot_;
        }
        void SetX0(double x0)
        {
            x0_ = x0;
        }
        void SetY0(double y0)
        {
            y0_ = y0;
        }
        void SetH0(double h0)
        {
            h0_ = h0;
        }
        void SetS0(double s0)
        {
            s0_ = s0;
        }
        void SetCDot(double c_dot)
        {
            c_dot_ = c_dot;
        }
        void   Print() const;
        void   EvaluateDS(double ds, double *x, double *y, double *h) const;
        double EvaluateCurvatureDS(double ds) const;
        void   SetX(double x);
        void   SetY(double y);
        void   SetHdg(double h);

        ClothoidType clothoid_type_;
        Arc          arc_;
        Line         line_;

    private:
        double curv_start_;
        double curv_end_;
        double c_dot_;
        double x0_;  // 0 if spiral starts with curvature = 0
        double y0_;  // 0 if spiral starts with curvature = 0
        double h0_;  // 0 if spiral starts with curvature = 0
        double s0_;  // 0 if spiral starts with curvature = 0
    };

    class Poly3 : public Geometry
    {
    public:
        Poly3() : umax_(0.0)
        {
        }
        Poly3(double s, double x, double y, double hdg, double length, double a, double b, double c, double d);
        ~Poly3(){};

        void SetUMax(double umax)
        {
            umax_ = umax;
        }
        double GetUMax() const
        {
            return umax_;
        }
        void       Print() const;
        Polynomial GetPoly3() const
        {
            return poly3_;
        }
        void   EvaluateDS(double ds, double *x, double *y, double *h) const;
        double EvaluateCurvatureDS(double ds) const;

        Polynomial poly3_;

    private:
        double umax_;
        void   EvaluateDSLocal(double ds, double &u, double &v) const;
    };

    class ParamPoly3 : public Geometry
    {
    public:
        enum PRangeType
        {
            P_RANGE_UNKNOWN,
            P_RANGE_NORMALIZED,
            P_RANGE_ARC_LENGTH
        };

        ParamPoly3()
        {
        }
        ParamPoly3(double     s,
                   double     x,
                   double     y,
                   double     hdg,
                   double     length,
                   double     aU,
                   double     bU,
                   double     cU,
                   double     dU,
                   double     aV,
                   double     bV,
                   double     cV,
                   double     dV,
                   PRangeType p_range)
            : Geometry(s, x, y, hdg, length, GeometryType::GEOMETRY_TYPE_PARAM_POLY3)
        {
            poly3U_.Set(aU, bU, cU, dU, p_range == PRangeType::P_RANGE_NORMALIZED ? 1.0 / length : 1.0);
            poly3V_.Set(aV, bV, cV, dV, p_range == PRangeType::P_RANGE_NORMALIZED ? 1.0 / length : 1.0);
            calcS2PMap(p_range);
        }
        ~ParamPoly3(){};

        void       Print() const;
        Polynomial GetPoly3U() const
        {
            return poly3U_;
        }
        Polynomial GetPoly3V() const
        {
            return poly3V_;
        }
        void   EvaluateDS(double ds, double *x, double *y, double *h) const;
        double EvaluateCurvatureDS(double ds) const;
        void   calcS2PMap(PRangeType p_range);
        double s2p_map_[PARAMPOLY3_STEPS + 1][2];
        double S2P(double s) const;

        Polynomial poly3U_;
        Polynomial poly3V_;
    };

    class Elevation
    {
    public:
        Elevation() : s_(0.0), length_(0.0)
        {
        }
        Elevation(double s, double a, double b, double c, double d) : s_(s), length_(0)
        {
            poly3_.Set(a, b, c, d);
        }
        ~Elevation(){};

        double GetS() const
        {
            return s_;
        }
        void SetLength(double length)
        {
            length_ = length;
        }
        double GetLength() const
        {
            return length_;
        }
        void Print() const;

        Polynomial poly3_;

    private:
        double s_;
        double length_;
    };

    typedef enum
    {
        NONE        = 0,
        SUCCESSOR   = 1,
        PREDECESSOR = -1
    } LinkType;

    class LaneLink
    {
    public:
        LaneLink(LinkType type, int id) : type_(type), id_(id)
        {
        }

        LinkType GetType() const
        {
            return type_;
        }
        int GetId() const
        {
            return id_;
        }
        void Print() const;

    private:
        LinkType type_;
        int      id_;
    };

    class LaneWidth
    {
    public:
        LaneWidth(double s_offset, double a, double b, double c, double d) : s_offset_(s_offset)
        {
            poly3_.Set(a, b, c, d);
        }

        double GetSOffset() const
        {
            return s_offset_;
        }
        void Print() const;

        Polynomial poly3_;

    private:
        double s_offset_;
    };

    class LaneBoundaryOSI
    {
    public:
        LaneBoundaryOSI(int gbid) : global_id_(gbid)
        {
        }
        ~LaneBoundaryOSI(){};
        void SetGlobalId();
        int  GetGlobalId() const
        {
            return global_id_;
        }
        OSIPoints *GetOSIPoints()
        {
            return &osi_points_;
        }
        OSIPoints osi_points_;

    private:
        int global_id_;  // Unique ID for OSI
    };

    struct RoadMarkInfo
    {
        int roadmark_idx_;
        int roadmarkline_idx_;
    };

    enum class RoadMarkColor
    {
        UNDEFINED,
        STANDARD_COLOR,  // equivalent to white
        BLUE,
        GREEN,
        RED,
        WHITE,
        YELLOW,
        UNKNOWN,
    };

    class LaneRoadMarkTypeLine
    {
    public:
        enum RoadMarkTypeLineRule
        {
            NO_PASSING,
            CAUTION,
            NONE
        };

        LaneRoadMarkTypeLine(double               length,
                             double               space,
                             double               t_offset,
                             double               s_offset,
                             RoadMarkTypeLineRule rule,
                             double               width,
                             RoadMarkColor        color = RoadMarkColor::UNDEFINED)
            : length_(length),
              space_(space),
              t_offset_(t_offset),
              s_offset_(s_offset),
              rule_(rule),
              width_(width),
              color_(color)
        {
        }
        ~LaneRoadMarkTypeLine(){};
        double GetSOffset() const
        {
            return s_offset_;
        }
        double GetTOffset() const
        {
            return t_offset_;
        }
        double GetLength() const
        {
            return length_;
        }
        double GetSpace() const
        {
            return space_;
        }
        double GetWidth() const
        {
            return width_;
        }
        OSIPoints *GetOSIPoints()
        {
            return &osi_points_;
        }
        OSIPoints osi_points_;
        void      SetGlobalId();
        int       GetGlobalId() const
        {
            return global_id_;
        }
        RoadMarkColor GetColor() const
        {
            return color_;
        }

    private:
        double               length_;
        double               space_;
        double               t_offset_;
        double               s_offset_;
        RoadMarkTypeLineRule rule_;
        double               width_;
        int                  global_id_;  // Unique ID for OSI
        RoadMarkColor        color_;      // if set, supersedes setting in <RoadMark>
    };

    class LaneRoadMarkExplicitLine
    {
    public:
        LaneRoadMarkExplicitLine(double length, double t_offset, double s_offset, LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule, double width)
            : length_(length),
              t_offset_(t_offset),
              s_offset_(s_offset),
              rule_(rule),
              width_(width)
        {
        }
        ~LaneRoadMarkExplicitLine(){};
        double GetSOffset() const
        {
            return s_offset_;
        }
        double GetTOffset() const
        {
            return t_offset_;
        }
        double GetLength() const
        {
            return length_;
        }
        double GetWidth() const
        {
            return width_;
        }
        OSIPoints *GetOSIPoints()
        {
            return &osi_points_;
        }
        OSIPoints osi_points_;
        void      SetGlobalId();
        int       GetGlobalId() const
        {
            return global_id_;
        }

    private:
        double                                     length_;
        double                                     t_offset_;
        double                                     s_offset_;
        LaneRoadMarkTypeLine::RoadMarkTypeLineRule rule_;
        double                                     width_;
        int                                        global_id_;  // Unique ID for OSI
    };

    class LaneRoadMarkType
    {
    public:
        LaneRoadMarkType(std::string name, double width) : name_(name), width_(width)
        {
        }

        void        AddLine(std::shared_ptr<LaneRoadMarkTypeLine> lane_roadMarkTypeLine);
        std::string GetName() const
        {
            return name_;
        }
        double GetWidth() const
        {
            return width_;
        }
        LaneRoadMarkTypeLine *GetLaneRoadMarkTypeLineByIdx(int idx) const;
        int                   GetNumberOfRoadMarkTypeLines() const
        {
            return (int)lane_roadMarkTypeLine_.size();
        }

    private:
        std::string                                        name_;
        double                                             width_;
        std::vector<std::shared_ptr<LaneRoadMarkTypeLine>> lane_roadMarkTypeLine_;
    };

    class LaneRoadMarkExplicit
    {
    public:
        LaneRoadMarkExplicit()
        {
        }

        void                      AddLine(std::shared_ptr<LaneRoadMarkExplicitLine> lane_roadMarkExplicitLine);
        LaneRoadMarkExplicitLine *GetLaneRoadMarkExplicitLineByIdx(int idx) const;
        int                       GetNumberOfLaneRoadMarkExplicitLines() const
        {
            return (int)lane_roadMarkExplicitLine_.size();
        }

    private:
        std::vector<std::shared_ptr<LaneRoadMarkExplicitLine>> lane_roadMarkExplicitLine_;
    };

    class LaneRoadMark
    {
    public:
        enum RoadMarkType
        {
            NONE_TYPE     = 1,
            SOLID         = 2,
            BROKEN        = 3,
            SOLID_SOLID   = 4,
            SOLID_BROKEN  = 5,
            BROKEN_SOLID  = 6,
            BROKEN_BROKEN = 7,
            BOTTS_DOTS    = 8,
            GRASS         = 9,
            CURB          = 10
        };

        enum RoadMarkWeight
        {
            STANDARD,
            BOLD
        };

        enum RoadMarkMaterial
        {
            STANDARD_MATERIAL  // only "standard" is available for now
        };

        enum RoadMarkLaneChange
        {
            INCREASE,
            DECREASE,
            BOTH,
            NONE_LANECHANGE
        };

        LaneRoadMark(double             s_offset,
                     RoadMarkType       type,
                     RoadMarkWeight     weight,
                     RoadMarkColor      color,
                     RoadMarkMaterial   material,
                     RoadMarkLaneChange lane_change,
                     double             width,
                     double             height)
            : s_offset_(s_offset),
              type_(type),
              weight_(weight),
              color_(color),
              material_(material),
              lane_change_(lane_change),
              width_(width),
              height_(height)
        {
        }

        void AddType(std::shared_ptr<LaneRoadMarkType> lane_roadMarkType);

        void AddExplicit(std::shared_ptr<LaneRoadMarkExplicit> lane_roadMarkExplicit);

        double GetSOffset() const
        {
            return s_offset_;
        }
        double GetWidth() const
        {
            return width_;
        }
        double GetHeight() const
        {
            return height_;
        }
        RoadMarkType GetType() const
        {
            return type_;
        }
        RoadMarkWeight GetWeight() const
        {
            return weight_;
        }
        RoadMarkColor GetColor() const
        {
            return color_;
        }
        RoadMarkMaterial GetMaterial() const
        {
            return material_;
        }
        RoadMarkLaneChange GetLaneChange() const
        {
            return lane_change_;
        }

        int GetNumberOfRoadMarkTypes() const
        {
            return (int)lane_roadMarkType_.size();
        }
        LaneRoadMarkType *GetLaneRoadMarkTypeByIdx(int idx) const;

        int GetNumberOfRoadMarkExplicit() const
        {
            return (int)lane_roadMarkExplicit_.size();
        }
        LaneRoadMarkExplicit *GetLaneRoadMarkExplicitByIdx(int idx) const;

        static RoadMarkColor ParseColor(pugi::xml_node node);
        static std::string   RoadMarkColor2Str(RoadMarkColor color);

    private:
        double                                             s_offset_;
        RoadMarkType                                       type_;
        RoadMarkWeight                                     weight_;
        RoadMarkColor                                      color_;
        RoadMarkMaterial                                   material_;
        RoadMarkLaneChange                                 lane_change_;
        double                                             width_;
        double                                             height_;
        std::vector<std::shared_ptr<LaneRoadMarkType>>     lane_roadMarkType_;
        std::vector<std::shared_ptr<LaneRoadMarkExplicit>> lane_roadMarkExplicit_;
    };

    class LaneOffset
    {
    public:
        LaneOffset() : s_(0.0), length_(0.0)
        {
        }
        LaneOffset(double s, double a, double b, double c, double d) : s_(s), length_(0.0)
        {
            polynomial_.Set(a, b, c, d);
        }
        ~LaneOffset()
        {
        }

        void Set(double s, double a, double b, double c, double d)
        {
            s_ = s;
            polynomial_.Set(a, b, c, d);
        }
        void SetLength(double length)
        {
            length_ = length;
        }
        double GetS() const
        {
            return s_;
        }
        Polynomial GetPolynomial() const
        {
            return polynomial_;
        }
        double GetLength() const
        {
            return length_;
        }
        double GetLaneOffset(double s) const;
        double GetLaneOffsetPrim(double s) const;
        void   Print() const;

    private:
        Polynomial polynomial_;
        double     s_;
        double     length_;
    };

    class Lane
    {
    public:
        enum LanePosition
        {
            LANE_POS_CENTER,
            LANE_POS_LEFT,
            LANE_POS_RIGHT
        };

        typedef enum
        {
            LANE_TYPE_NONE            = (1 << 0),
            LANE_TYPE_DRIVING         = (1 << 1),
            LANE_TYPE_STOP            = (1 << 2),
            LANE_TYPE_SHOULDER        = (1 << 3),
            LANE_TYPE_BIKING          = (1 << 4),
            LANE_TYPE_SIDEWALK        = (1 << 5),
            LANE_TYPE_BORDER          = (1 << 6),
            LANE_TYPE_RESTRICTED      = (1 << 7),
            LANE_TYPE_PARKING         = (1 << 8),
            LANE_TYPE_BIDIRECTIONAL   = (1 << 9),
            LANE_TYPE_MEDIAN          = (1 << 10),
            LANE_TYPE_SPECIAL1        = (1 << 11),
            LANE_TYPE_SPECIAL2        = (1 << 12),
            LANE_TYPE_SPECIAL3        = (1 << 13),
            LANE_TYPE_ROADMARKS       = (1 << 14),
            LANE_TYPE_TRAM            = (1 << 15),
            LANE_TYPE_RAIL            = (1 << 16),
            LANE_TYPE_ENTRY           = (1 << 17),
            LANE_TYPE_EXIT            = (1 << 18),
            LANE_TYPE_OFF_RAMP        = (1 << 19),
            LANE_TYPE_ON_RAMP         = (1 << 20),
            LANE_TYPE_CURB            = (1 << 21),
            LANE_TYPE_CONNECTING_RAMP = (1 << 22),
            LANE_TYPE_REFERENCE_LINE  = (1 << 0),
            LANE_TYPE_ANY_DRIVING =
                LANE_TYPE_DRIVING | LANE_TYPE_ENTRY | LANE_TYPE_EXIT | LANE_TYPE_OFF_RAMP | LANE_TYPE_ON_RAMP | LANE_TYPE_BIDIRECTIONAL,
            LANE_TYPE_ANY_ROAD = LANE_TYPE_ANY_DRIVING | LANE_TYPE_RESTRICTED | LANE_TYPE_STOP,
            LANE_TYPE_ANY      = (-1)
        } LaneType;

        // Construct & Destruct
        Lane() : id_(0), type_(LaneType::LANE_TYPE_NONE), level_(0), global_id_(0), lane_boundary_(0), road_edge_(false)
        {
        }
        Lane(int id, Lane::LaneType type) : id_(id), type_(type), level_(1), global_id_(0), lane_boundary_(0), road_edge_(false)
        {
        }
        ~Lane()
        {
            for (size_t i = 0; i < link_.size(); i++)
            {
                delete link_[i];
            }
            link_.clear();

            for (size_t i = 0; i < lane_width_.size(); i++)
            {
                delete lane_width_[i];
            }
            lane_width_.clear();

            for (size_t i = 0; i < lane_roadMark_.size(); i++)
            {
                delete lane_roadMark_[i];
            }
            lane_roadMark_.clear();

            if (lane_boundary_)
            {
                delete lane_boundary_;
                lane_boundary_ = 0;
            }
        }

        // Base Get Functions
        int GetId() const
        {
            return id_;
        }
        LaneType GetLaneType() const
        {
            return type_;
        }
        int GetGlobalId() const
        {
            return global_id_;
        }

        // Add Functions
        void AddLink(LaneLink *lane_link)
        {
            link_.push_back(lane_link);
        }
        void AddLaneWidth(LaneWidth *lane_width);
        void AddLaneRoadMark(LaneRoadMark *lane_roadMark);

        // Get Functions
        int GetNumberOfRoadMarks() const
        {
            return (int)lane_roadMark_.size();
        }
        int GetNumberOfLinks() const
        {
            return (int)link_.size();
        }
        int GetNumberOfLaneWidths() const
        {
            return (int)lane_width_.size();
        }

        LaneLink     *GetLink(LinkType type) const;
        LaneWidth    *GetWidthByIndex(int index) const;
        LaneWidth    *GetWidthByS(double s) const;
        LaneRoadMark *GetLaneRoadMarkByIdx(int idx) const;

        RoadMarkInfo GetRoadMarkInfoByS(int track_id, int lane_id, double s) const;
        OSIPoints   *GetOSIPoints()
        {
            return &osi_points_;
        }
        std::vector<int> GetLineGlobalIds() const;
        LaneBoundaryOSI *GetLaneBoundary() const
        {
            return lane_boundary_;
        }
        int GetLaneBoundaryGlobalId() const;

        // Set Functions
        void SetGlobalId();
        void SetLaneBoundary(LaneBoundaryOSI *lane_boundary);

        // Others
        bool IsType(Lane::LaneType type);
        bool IsCenter();
        bool IsDriving();
        bool IsOSIIntersection()
        {
            return osiintersection_ > 0;
        }
        void SetOSIIntersection(int is_osi_intersection)
        {
            osiintersection_ = is_osi_intersection;
        }
        int GetOSIIntersectionId()
        {
            return osiintersection_;
        }
        void      Print() const;
        OSIPoints osi_points_;
        void      SetRoadEdge(bool value)
        {
            road_edge_ = value;
        }
        bool IsRoadEdge() const
        {
            return road_edge_;
        }

    private:
        int                         id_;               // center = 0, left > 0, right < 0
        int                         global_id_;        // Unique ID for OSI
        int                         osiintersection_;  // flag to see if the lane is part of an osi-lane section or not
        LaneType                    type_;
        int                         level_;  // boolean, true = keep lane on level
        std::vector<LaneLink *>     link_;
        std::vector<LaneWidth *>    lane_width_;
        std::vector<LaneRoadMark *> lane_roadMark_;
        LaneBoundaryOSI            *lane_boundary_;
        bool                        road_edge_;  // indicates whether this is edge of the paved road (used for OSI ROAD_EDGE)
    };

    class LaneSection
    {
    public:
        LaneSection(double s) : s_(s), length_(0)
        {
        }
        ~LaneSection()
        {
            for (size_t i = 0; i < lane_.size(); i++)
            {
                delete lane_[i];
            }
            lane_.clear();
        }
        void   AddLane(Lane *lane);
        double GetS() const
        {
            return s_;
        }
        Lane  *GetLaneByIdx(int idx) const;
        Lane  *GetLaneById(int id) const;
        int    GetLaneIdByIdx(int idx) const;
        int    GetLaneIdxById(int id) const;
        bool   IsOSILaneById(int id) const;
        int    GetLaneGlobalIdByIdx(int idx) const;
        int    GetLaneGlobalIdById(int id) const;
        double GetOuterOffset(double s, int lane_id) const;
        double GetWidth(double s, int lane_id) const;

        /**
        Get index of closest lane wrt given constraints
        @param s distance along the road segment
        @param lane_id lane specifier, starting from center -1, -2, ... is on the right side, 1, 2... on the left
        */
        int GetClosestLaneIdx(double  s,
                              double  t,
                              int     side,
                              double &offset,
                              bool    noZeroWidth,
                              int     laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING) const;

        /**
        Get lateral position of lane center, from road reference lane (lane id=0)
        Example: If lane id 1 is 5 m wide and lane id 2 is 4 m wide, then
                        lane 1 center offset is 5/2 = 2.5 and lane 2 center offset is 5 + 4/2 = 7
        @param s distance along the road segment
        @param lane_id lane specifier, starting from center -1, -2, ... is on the right side, 1, 2... on the left
        @return Lateral position of lane center
        */
        double GetCenterOffset(double s, int lane_id) const;
        double GetOuterOffsetHeading(double s, int lane_id) const;
        double GetCenterOffsetHeading(double s, int lane_id) const;
        double GetLength() const
        {
            return length_;
        }
        int GetNumberOfLanes() const
        {
            return (int)lane_.size();
        }
        int  GetNumberOfDrivingLanes() const;
        int  GetNumberOfDrivingLanesSide(int side) const;
        int  GetNUmberOfLanesRight() const;
        int  GetNUmberOfLanesLeft() const;
        void SetLength(double length)
        {
            length_ = length;
        }
        int    GetConnectingLaneId(int incoming_lane_id, LinkType link_type) const;
        double GetWidthBetweenLanes(int lane_id1, int lane_id2, double s) const;
        double GetOffsetBetweenLanes(int lane_id1, int lane_id2, double s) const;
        void   Print() const;

    private:
        double              s_;
        double              length_;
        std::vector<Lane *> lane_;
    };

    enum ContactPointType
    {
        CONTACT_POINT_UNDEFINED,
        CONTACT_POINT_START,
        CONTACT_POINT_END,
        CONTACT_POINT_JUNCTION,  // No contact point for element type junction
    };

    class RoadLink
    {
    public:
        typedef enum
        {
            ELEMENT_TYPE_UNKNOWN,
            ELEMENT_TYPE_ROAD,
            ELEMENT_TYPE_JUNCTION,
        } ElementType;

        RoadLink() : type_(NONE), element_id_(-1), element_type_(ELEMENT_TYPE_UNKNOWN), contact_point_type_(CONTACT_POINT_UNDEFINED)
        {
        }
        RoadLink(LinkType type, ElementType element_type, int element_id, ContactPointType contact_point_type)
            : type_(type),
              element_id_(element_id),
              element_type_(element_type),
              contact_point_type_(contact_point_type)
        {
        }
        RoadLink(LinkType type, pugi::xml_node node);
        bool operator==(RoadLink &rhs);

        int GetElementId() const
        {
            return element_id_;
        }
        LinkType GetType() const
        {
            return type_;
        }
        RoadLink::ElementType GetElementType() const
        {
            return element_type_;
        }
        ContactPointType GetContactPointType() const
        {
            return contact_point_type_;
        }

        void Print() const;

    private:
        LinkType         type_;
        int              element_id_;
        ElementType      element_type_;
        ContactPointType contact_point_type_;
    };

    struct LaneInfo
    {
        int lane_section_idx_;
        int lane_id_;
    };

    typedef struct
    {
        int fromLane_;
        int toLane_;
    } ValidityRecord;

    class RoadObject
    {
    public:
        enum Orientation
        {
            POSITIVE,
            NEGATIVE,
            NONE,
        };

        RoadObject() : x_(0.0), y_(0.0), z_(0.0), h_(0.0)
        {
        }
        RoadObject(double x, double y, double z, double h) : x_(x), y_(y), z_(z), h_(h)
        {
        }

        double GetX() const
        {
            return x_;
        }  // X coordinate of sign position
        double GetY() const
        {
            return y_;
        }  // Y coordinate of sign position
        double GetZ() const
        {
            return z_;
        }  // Z coordinate of road level at sign X, Y position
        double GetH() const
        {
            return h_;
        }  // sign yaw rotation including "orientation" (+/-) but excluding h_offset

        std::vector<ValidityRecord> validity_;
        double                      x_;
        double                      y_;
        double                      z_;
        double                      h_;
    };

    class Signal : public RoadObject
    {
    public:
        enum OSIType : int
        {
            TYPE_UNKNOWN                                                          = 0,
            TYPE_OTHER                                                            = 1,
            TYPE_DANGER_SPOT                                                      = 2,
            TYPE_ZEBRA_CROSSING                                                   = 87,
            TYPE_FLIGHT                                                           = 110,
            TYPE_CATTLE                                                           = 200,
            TYPE_HORSE_RIDERS                                                     = 197,
            TYPE_AMPHIBIANS                                                       = 188,
            TYPE_FALLING_ROCKS                                                    = 96,
            TYPE_SNOW_OR_ICE                                                      = 94,
            TYPE_LOOSE_GRAVEL                                                     = 97,
            TYPE_WATERSIDE                                                        = 102,
            TYPE_CLEARANCE                                                        = 210,
            TYPE_MOVABLE_BRIDGE                                                   = 101,
            TYPE_RIGHT_BEFORE_LEFT_NEXT_INTERSECTION                              = 3,
            TYPE_TURN_LEFT                                                        = 4,
            TYPE_TURN_RIGHT                                                       = 5,
            TYPE_DOUBLE_TURN_LEFT                                                 = 6,
            TYPE_DOUBLE_TURN_RIGHT                                                = 7,
            TYPE_HILL_DOWNWARDS                                                   = 8,
            TYPE_HILL_UPWARDS                                                     = 9,
            TYPE_UNEVEN_ROAD                                                      = 93,
            TYPE_ROAD_SLIPPERY_WET_OR_DIRTY                                       = 95,
            TYPE_SIDE_WINDS                                                       = 98,
            TYPE_ROAD_NARROWING                                                   = 10,
            TYPE_ROAD_NARROWING_RIGHT                                             = 12,
            TYPE_ROAD_NARROWING_LEFT                                              = 11,
            TYPE_ROAD_WORKS                                                       = 13,
            TYPE_TRAFFIC_QUEUES                                                   = 100,
            TYPE_TWO_WAY_TRAFFIC                                                  = 14,
            TYPE_ATTENTION_TRAFFIC_LIGHT                                          = 15,
            TYPE_PEDESTRIANS                                                      = 103,
            TYPE_CHILDREN_CROSSING                                                = 106,
            TYPE_CYCLE_ROUTE                                                      = 107,
            TYPE_DEER_CROSSING                                                    = 109,
            TYPE_UNGATED_LEVEL_CROSSING                                           = 144,
            TYPE_LEVEL_CROSSING_MARKER                                            = 112,
            TYPE_RAILWAY_TRAFFIC_PRIORITY                                         = 135,
            TYPE_GIVE_WAY                                                         = 16,
            TYPE_STOP                                                             = 17,
            TYPE_PRIORITY_TO_OPPOSITE_DIRECTION                                   = 18,
            TYPE_PRIORITY_TO_OPPOSITE_DIRECTION_UPSIDE_DOWN                       = 19,
            TYPE_PRESCRIBED_LEFT_TURN                                             = 20,
            TYPE_PRESCRIBED_RIGHT_TURN                                            = 21,
            TYPE_PRESCRIBED_STRAIGHT                                              = 22,
            TYPE_PRESCRIBED_RIGHT_WAY                                             = 24,
            TYPE_PRESCRIBED_LEFT_WAY                                              = 23,
            TYPE_PRESCRIBED_RIGHT_TURN_AND_STRAIGHT                               = 26,
            TYPE_PRESCRIBED_LEFT_TURN_AND_STRAIGHT                                = 25,
            TYPE_PRESCRIBED_LEFT_TURN_AND_RIGHT_TURN                              = 27,
            TYPE_PRESCRIBED_LEFT_TURN_RIGHT_TURN_AND_STRAIGHT                     = 28,
            TYPE_ROUNDABOUT                                                       = 29,
            TYPE_ONEWAY_LEFT                                                      = 30,
            TYPE_ONEWAY_RIGHT                                                     = 31,
            TYPE_PASS_LEFT                                                        = 32,
            TYPE_PASS_RIGHT                                                       = 33,
            TYPE_SIDE_LANE_OPEN_FOR_TRAFFIC                                       = 128,
            TYPE_SIDE_LANE_CLOSED_FOR_TRAFFIC                                     = 129,
            TYPE_SIDE_LANE_CLOSING_FOR_TRAFFIC                                    = 130,
            TYPE_BUS_STOP                                                         = 137,
            TYPE_TAXI_STAND                                                       = 138,
            TYPE_BICYCLES_ONLY                                                    = 145,
            TYPE_HORSE_RIDERS_ONLY                                                = 146,
            TYPE_PEDESTRIANS_ONLY                                                 = 147,
            TYPE_BICYCLES_PEDESTRIANS_SHARED_ONLY                                 = 148,
            TYPE_BICYCLES_PEDESTRIANS_SEPARATED_LEFT_ONLY                         = 149,
            TYPE_BICYCLES_PEDESTRIANS_SEPARATED_RIGHT_ONLY                        = 150,
            TYPE_PEDESTRIAN_ZONE_BEGIN                                            = 151,
            TYPE_PEDESTRIAN_ZONE_END                                              = 152,
            TYPE_BICYCLE_ROAD_BEGIN                                               = 153,
            TYPE_BICYCLE_ROAD_END                                                 = 154,
            TYPE_BUS_LANE                                                         = 34,
            TYPE_BUS_LANE_BEGIN                                                   = 35,
            TYPE_BUS_LANE_END                                                     = 36,
            TYPE_ALL_PROHIBITED                                                   = 37,
            TYPE_MOTORIZED_MULTITRACK_PROHIBITED                                  = 38,
            TYPE_TRUCKS_PROHIBITED                                                = 39,
            TYPE_BICYCLES_PROHIBITED                                              = 40,
            TYPE_MOTORCYCLES_PROHIBITED                                           = 41,
            TYPE_MOPEDS_PROHIBITED                                                = 155,
            TYPE_HORSE_RIDERS_PROHIBITED                                          = 156,
            TYPE_HORSE_CARRIAGES_PROHIBITED                                       = 157,
            TYPE_CATTLE_PROHIBITED                                                = 158,
            TYPE_BUSES_PROHIBITED                                                 = 159,
            TYPE_CARS_PROHIBITED                                                  = 160,
            TYPE_CARS_TRAILERS_PROHIBITED                                         = 161,
            TYPE_TRUCKS_TRAILERS_PROHIBITED                                       = 162,
            TYPE_TRACTORS_PROHIBITED                                              = 163,
            TYPE_PEDESTRIANS_PROHIBITED                                           = 42,
            TYPE_MOTOR_VEHICLES_PROHIBITED                                        = 43,
            TYPE_HAZARDOUS_GOODS_VEHICLES_PROHIBITED                              = 164,
            TYPE_OVER_WEIGHT_VEHICLES_PROHIBITED                                  = 165,
            TYPE_VEHICLES_AXLE_OVER_WEIGHT_PROHIBITED                             = 166,
            TYPE_VEHICLES_EXCESS_WIDTH_PROHIBITED                                 = 167,
            TYPE_VEHICLES_EXCESS_HEIGHT_PROHIBITED                                = 168,
            TYPE_VEHICLES_EXCESS_LENGTH_PROHIBITED                                = 169,
            TYPE_DO_NOT_ENTER                                                     = 44,
            TYPE_SNOW_CHAINS_REQUIRED                                             = 170,
            TYPE_WATER_POLLUTANT_VEHICLES_PROHIBITED                              = 171,
            TYPE_ENVIRONMENTAL_ZONE_BEGIN                                         = 45,
            TYPE_ENVIRONMENTAL_ZONE_END                                           = 46,
            TYPE_NO_U_TURN_LEFT                                                   = 47,
            TYPE_NO_U_TURN_RIGHT                                                  = 48,
            TYPE_PRESCRIBED_U_TURN_LEFT                                           = 49,
            TYPE_PRESCRIBED_U_TURN_RIGHT                                          = 50,
            TYPE_MINIMUM_DISTANCE_FOR_TRUCKS                                      = 51,
            TYPE_SPEED_LIMIT_BEGIN                                                = 52,
            TYPE_SPEED_LIMIT_ZONE_BEGIN                                           = 53,
            TYPE_SPEED_LIMIT_ZONE_END                                             = 54,
            TYPE_MINIMUM_SPEED_BEGIN                                              = 55,
            TYPE_OVERTAKING_BAN_BEGIN                                             = 56,
            TYPE_OVERTAKING_BAN_FOR_TRUCKS_BEGIN                                  = 57,
            TYPE_SPEED_LIMIT_END                                                  = 58,
            TYPE_MINIMUM_SPEED_END                                                = 59,
            TYPE_OVERTAKING_BAN_END                                               = 60,
            TYPE_OVERTAKING_BAN_FOR_TRUCKS_END                                    = 61,
            TYPE_ALL_RESTRICTIONS_END                                             = 62,
            TYPE_NO_STOPPING                                                      = 63,
            TYPE_NO_PARKING                                                       = 64,
            TYPE_NO_PARKING_ZONE_BEGIN                                            = 65,
            TYPE_NO_PARKING_ZONE_END                                              = 66,
            TYPE_RIGHT_OF_WAY_NEXT_INTERSECTION                                   = 67,
            TYPE_RIGHT_OF_WAY_BEGIN                                               = 68,
            TYPE_RIGHT_OF_WAY_END                                                 = 69,
            TYPE_PRIORITY_OVER_OPPOSITE_DIRECTION                                 = 70,
            TYPE_PRIORITY_OVER_OPPOSITE_DIRECTION_UPSIDE_DOWN                     = 71,
            TYPE_TOWN_BEGIN                                                       = 72,
            TYPE_TOWN_END                                                         = 73,
            TYPE_CAR_PARKING                                                      = 74,
            TYPE_CAR_PARKING_ZONE_BEGIN                                           = 75,
            TYPE_CAR_PARKING_ZONE_END                                             = 76,
            TYPE_SIDEWALK_HALF_PARKING_LEFT                                       = 172,
            TYPE_SIDEWALK_HALF_PARKING_RIGHT                                      = 173,
            TYPE_SIDEWALK_PARKING_LEFT                                            = 174,
            TYPE_SIDEWALK_PARKING_RIGHT                                           = 175,
            TYPE_SIDEWALK_PERPENDICULAR_HALF_PARKING_LEFT                         = 176,
            TYPE_SIDEWALK_PERPENDICULAR_HALF_PARKING_RIGHT                        = 177,
            TYPE_SIDEWALK_PERPENDICULAR_PARKING_LEFT                              = 178,
            TYPE_SIDEWALK_PERPENDICULAR_PARKING_RIGHT                             = 179,
            TYPE_LIVING_STREET_BEGIN                                              = 77,
            TYPE_LIVING_STREET_END                                                = 78,
            TYPE_TUNNEL                                                           = 79,
            TYPE_EMERGENCY_STOPPING_LEFT                                          = 80,
            TYPE_EMERGENCY_STOPPING_RIGHT                                         = 81,
            TYPE_HIGHWAY_BEGIN                                                    = 82,
            TYPE_HIGHWAY_END                                                      = 83,
            TYPE_EXPRESSWAY_BEGIN                                                 = 84,
            TYPE_EXPRESSWAY_END                                                   = 85,
            TYPE_NAMED_HIGHWAY_EXIT                                               = 183,
            TYPE_NAMED_EXPRESSWAY_EXIT                                            = 184,
            TYPE_NAMED_ROAD_EXIT                                                  = 185,
            TYPE_HIGHWAY_EXIT                                                     = 86,
            TYPE_EXPRESSWAY_EXIT                                                  = 186,
            TYPE_ONEWAY_STREET                                                    = 187,
            TYPE_CROSSING_GUARDS                                                  = 189,
            TYPE_DEADEND                                                          = 190,
            TYPE_DEADEND_EXCLUDING_DESIGNATED_ACTORS                              = 191,
            TYPE_FIRST_AID_STATION                                                = 194,
            TYPE_POLICE_STATION                                                   = 195,
            TYPE_TELEPHONE                                                        = 196,
            TYPE_FILLING_STATION                                                  = 198,
            TYPE_HOTEL                                                            = 201,
            TYPE_INN                                                              = 202,
            TYPE_KIOSK                                                            = 203,
            TYPE_TOILET                                                           = 204,
            TYPE_CHAPEL                                                           = 205,
            TYPE_TOURIST_INFO                                                     = 206,
            TYPE_REPAIR_SERVICE                                                   = 207,
            TYPE_PEDESTRIAN_UNDERPASS                                             = 208,
            TYPE_PEDESTRIAN_BRIDGE                                                = 209,
            TYPE_CAMPER_PLACE                                                     = 213,
            TYPE_ADVISORY_SPEED_LIMIT_BEGIN                                       = 214,
            TYPE_ADVISORY_SPEED_LIMIT_END                                         = 215,
            TYPE_PLACE_NAME                                                       = 216,
            TYPE_TOURIST_ATTRACTION                                               = 217,
            TYPE_TOURIST_ROUTE                                                    = 218,
            TYPE_TOURIST_AREA                                                     = 219,
            TYPE_SHOULDER_NOT_PASSABLE_MOTOR_VEHICLES                             = 220,
            TYPE_SHOULDER_UNSAFE_TRUCKS_TRACTORS                                  = 221,
            TYPE_TOLL_BEGIN                                                       = 222,
            TYPE_TOLL_END                                                         = 223,
            TYPE_TOLL_ROAD                                                        = 224,
            TYPE_CUSTOMS                                                          = 225,
            TYPE_INTERNATIONAL_BORDER_INFO                                        = 226,
            TYPE_STREETLIGHT_RED_BAND                                             = 227,
            TYPE_FEDERAL_HIGHWAY_ROUTE_NUMBER                                     = 228,
            TYPE_HIGHWAY_ROUTE_NUMBER                                             = 229,
            TYPE_HIGHWAY_INTERCHANGE_NUMBER                                       = 230,
            TYPE_EUROPEAN_ROUTE_NUMBER                                            = 231,
            TYPE_FEDERAL_HIGHWAY_DIRECTION_LEFT                                   = 232,
            TYPE_FEDERAL_HIGHWAY_DIRECTION_RIGHT                                  = 233,
            TYPE_PRIMARY_ROAD_DIRECTION_LEFT                                      = 234,
            TYPE_PRIMARY_ROAD_DIRECTION_RIGHT                                     = 235,
            TYPE_SECONDARY_ROAD_DIRECTION_LEFT                                    = 236,
            TYPE_SECONDARY_ROAD_DIRECTION_RIGHT                                   = 237,
            TYPE_DIRECTION_DESIGNATED_ACTORS_LEFT                                 = 238,
            TYPE_DIRECTION_DESIGNATED_ACTORS_RIGHT                                = 239,
            TYPE_ROUTING_DESIGNATED_ACTORS                                        = 240,
            TYPE_DIRECTION_TO_HIGHWAY_LEFT                                        = 143,
            TYPE_DIRECTION_TO_HIGHWAY_RIGHT                                       = 108,
            TYPE_DIRECTION_TO_LOCAL_DESTINATION_LEFT                              = 127,
            TYPE_DIRECTION_TO_LOCAL_DESTINATION_RIGHT                             = 136,
            TYPE_CONSOLIDATED_DIRECTIONS                                          = 118,
            TYPE_STREET_NAME                                                      = 119,
            TYPE_DIRECTION_PREANNOUNCEMENT                                        = 120,
            TYPE_DIRECTION_PREANNOUNCEMENT_LANE_CONFIG                            = 121,
            TYPE_DIRECTION_PREANNOUNCEMENT_HIGHWAY_ENTRIES                        = 122,
            TYPE_HIGHWAY_ANNOUNCEMENT                                             = 123,
            TYPE_OTHER_ROAD_ANNOUNCEMENT                                          = 124,
            TYPE_HIGHWAY_ANNOUNCEMENT_TRUCK_STOP                                  = 125,
            TYPE_HIGHWAY_PREANNOUNCEMENT_DIRECTIONS                               = 126,
            TYPE_POLE_EXIT                                                        = 88,
            TYPE_HIGHWAY_DISTANCE_BOARD                                           = 180,
            TYPE_DETOUR_LEFT                                                      = 181,
            TYPE_DETOUR_RIGHT                                                     = 182,
            TYPE_NUMBERED_DETOUR                                                  = 131,
            TYPE_DETOUR_BEGIN                                                     = 132,
            TYPE_DETOUR_END                                                       = 133,
            TYPE_DETOUR_ROUTING_BOARD                                             = 134,
            TYPE_OPTIONAL_DETOUR                                                  = 111,
            TYPE_OPTIONAL_DETOUR_ROUTING                                          = 199,
            TYPE_ROUTE_RECOMMENDATION                                             = 211,
            TYPE_ROUTE_RECOMMENDATION_END                                         = 212,
            TYPE_ANNOUNCE_LANE_TRANSITION_LEFT                                    = 192,
            TYPE_ANNOUNCE_LANE_TRANSITION_RIGHT                                   = 193,
            TYPE_ANNOUNCE_RIGHT_LANE_END                                          = 90,
            TYPE_ANNOUNCE_LEFT_LANE_END                                           = 89,
            TYPE_ANNOUNCE_RIGHT_LANE_BEGIN                                        = 115,
            TYPE_ANNOUNCE_LEFT_LANE_BEGIN                                         = 116,
            TYPE_ANNOUNCE_LANE_CONSOLIDATION                                      = 117,
            TYPE_DETOUR_CITY_BLOCK                                                = 142,
            TYPE_GATE                                                             = 141,
            TYPE_POLE_WARNING                                                     = 91,
            TYPE_TRAFFIC_CONE                                                     = 140,
            TYPE_MOBILE_LANE_CLOSURE                                              = 139,
            TYPE_REFLECTOR_POST                                                   = 114,
            TYPE_DIRECTIONAL_BOARD_WARNING                                        = 113,
            TYPE_GUIDING_PLATE                                                    = 104,
            TYPE_GUIDING_PLATE_WEDGES                                             = 105,
            TYPE_PARKING_HAZARD                                                   = 99,
            TYPE_TRAFFIC_LIGHT_GREEN_ARROW                                        = 92,
            TrafficSign_MainSign_Classification_Type_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
            TrafficSign_MainSign_Classification_Type_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
        };

        Signal(double      s,
               double      t,
               int         id,
               std::string name,
               bool        dynamic,
               Orientation orientation,
               double      z_offset,
               std::string country,
               int         osi_type,
               std::string type,
               std::string subtype,
               std::string value_str_,
               std::string unit,
               double      height,
               double      width,
               std::string text,
               double      h_offset,
               double      pitch,
               double      roll,
               double      x,
               double      y,
               double      z,
               double      h);

        std::string GetName() const
        {
            return name_;
        }
        int GetId() const
        {
            return id_;
        }
        double GetS() const
        {
            return s_;
        }
        double GetT() const
        {
            return t_;
        }
        void SetLength(double length)
        {
            length_ = length;
        }
        double GetLength() const
        {
            return length_;
        }
        double GetHOffset() const
        {
            return h_offset_;
        }
        double GetZOffset() const
        {
            return z_offset_;
        }
        Orientation GetOrientation() const
        {
            return orientation_;
        }
        int GetOSIType() const
        {
            return osi_type_;
        }
        std::string GetType() const
        {
            return type_;
        }
        std::string GetSubType() const
        {
            return subtype_;
        }
        std::string GetCountry() const
        {
            return country_;
        }
        double GetHeight() const
        {
            return height_;
        }
        double GetWidth() const
        {
            return width_;
        }
        bool IsDynamic() const
        {
            return dynamic_;
        }
        double GetPitch() const
        {
            return pitch_;
        }
        double GetRoll() const
        {
            return roll_;
        }
        double GetValue() const
        {
            return value_;
        }
        std::string GetValueStr() const
        {
            return value_str_;
        }
        std::string GetUnit() const
        {
            return unit_;
        }
        std::string GetText() const
        {
            return text_;
        }
        static OSIType     GetOSITypeFromString(const std::string &type);
        static std::string GetCombinedTypeSubtypeValueStr(std::string type, std::string subtype, std::string value);

    private:
        double                                      s_;
        double                                      t_;
        int                                         id_;
        std::string                                 name_;
        bool                                        dynamic_;
        Orientation                                 orientation_;
        double                                      z_offset_;
        std::string                                 country_;
        int                                         osi_type_;
        std::string                                 type_;
        std::string                                 subtype_;
        std::string                                 value_str_;
        double                                      value_;
        std::string                                 unit_;
        double                                      height_;
        double                                      width_;
        std::string                                 text_;
        double                                      h_offset_;
        double                                      pitch_;
        double                                      roll_;
        double                                      length_;
        static const std::map<std::string, OSIType> types_mapping_;
    };

    class OutlineCorner
    {
    public:
        virtual void   GetPos(double &x, double &y, double &z)      = 0;
        virtual void   GetPosLocal(double &x, double &y, double &z) = 0;
        virtual double GetHeight()                                  = 0;
        virtual ~OutlineCorner()
        {
        }
    };

    class OutlineCornerRoad : public OutlineCorner
    {
    public:
        OutlineCornerRoad(int roadId, double s, double t, double dz, double height, double center_s, double center_t, double center_heading);
        void   GetPos(double &x, double &y, double &z) override;
        void   GetPosLocal(double &x, double &y, double &z) override;
        double GetHeight()
        {
            return height_;
        }

    private:
        int    roadId_;
        double s_, t_, dz_, height_, center_s_, center_t_, center_heading_;
    };

    class OutlineCornerLocal : public OutlineCorner
    {
    public:
        OutlineCornerLocal(int roadId, double s, double t, double u, double v, double zLocal, double height, double heading);
        void   GetPos(double &x, double &y, double &z) override;
        void   GetPosLocal(double &x, double &y, double &z) override;
        double GetHeight()
        {
            return height_;
        }

    private:
        int    roadId_;
        double s_, t_, u_, v_, zLocal_, height_, heading_;
    };

    class Outline
    {
    public:
        typedef enum
        {
            FILL_TYPE_GRASS,
            FILL_TYPE_CONCRETE,
            FILL_TYPE_COBBLE,
            FILL_TYPE_ASPHALT,
            FILL_TYPE_PAVEMENT,
            FILL_TYPE_GRAVEL,
            FILL_TYPE_SOIL,
            FILL_TYPE_UNDEFINED
        } FillType;

        int                          id_;
        FillType                     fillType_;
        bool                         closed_;
        std::vector<OutlineCorner *> corner_;

        Outline(int id, FillType fillType, bool closed) : id_(id), fillType_(fillType), closed_(closed)
        {
        }

        ~Outline()
        {
            for (size_t i = 0; i < corner_.size(); i++)
                delete (corner_[i]);
            corner_.clear();
        }

        void AddCorner(OutlineCorner *outlineCorner)
        {
            corner_.push_back(outlineCorner);
        }
    };

    class Repeat
    {
    public:
        double s_;
        double length_;
        double distance_;
        double tStart_;
        double tEnd_;
        double heightStart_;
        double heightEnd_;
        double zOffsetStart_;
        double zOffsetEnd_;
        double widthStart_;
        double widthEnd_;
        double lengthStart_;
        double lengthEnd_;
        double radiusStart_;
        double radiusEnd_;

        Repeat(double s,
               double length,
               double distance,
               double tStart,
               double tEnd,
               double heightStart,
               double heightEnd,
               double zOffsetStart,
               double zOffsetEnd)
            : s_(s),
              length_(length),
              distance_(distance),
              tStart_(tStart),
              tEnd_(tEnd),
              heightStart_(heightStart),
              heightEnd_(heightEnd),
              zOffsetStart_(zOffsetStart),
              zOffsetEnd_(zOffsetEnd),
              widthStart_(0.0),
              widthEnd_(0.0),
              lengthStart_(0.0),
              lengthEnd_(0.0),
              radiusStart_(0.0),
              radiusEnd_(0.0)
        {
        }

        void SetWidthStart(double widthStart)
        {
            widthStart_ = widthStart;
        }
        void SetWidthEnd(double widthEnd)
        {
            widthEnd_ = widthEnd;
        }
        void SetLengthStart(double lengthStart)
        {
            lengthStart_ = lengthStart;
        }
        void SetLengthEnd(double lengthEnd)
        {
            lengthEnd_ = lengthEnd;
        }
        void SetHeightStart(double heightStart)
        {
            heightStart_ = heightStart;
        }
        void SeHeightEnd(double heightEnd)
        {
            heightEnd_ = heightEnd;
        }
        double GetS() const
        {
            return s_;
        }
        double GetLength() const
        {
            return length_;
        }
        double GetDistance() const
        {
            return distance_;
        }
        double GetTStart() const
        {
            return tStart_;
        }
        double GetTEnd() const
        {
            return tEnd_;
        }
        double GetHeightStart() const
        {
            return heightStart_;
        }
        double GetHeightEnd() const
        {
            return heightEnd_;
        }
        double GetZOffsetStart() const
        {
            return zOffsetStart_;
        }
        double GetZOffsetEnd() const
        {
            return zOffsetEnd_;
        }
        double GetWidthStart() const
        {
            return widthStart_;
        }
        double GetWidthEnd() const
        {
            return widthEnd_;
        }
        double GetLengthStart() const
        {
            return lengthStart_;
        }
        double GetLengthEnd() const
        {
            return lengthEnd_;
        }
        double GetRadiusStart() const
        {
            return radiusStart_;
        }
        double GetRadiusEnd() const
        {
            return radiusEnd_;
        }
    };

    class RMObject : public RoadObject
    {
    public:
        enum class ObjectType
        {
            BARRIER,
            BIKE,
            BUILDING,
            BUS,
            CAR,
            CROSSWALK,
            GANTRY,
            MOTORBIKE,
            NONE,
            OBSTACLE,
            PARKINGSPACE,
            PATCH,
            PEDESTRIAN,
            POLE,
            RAILING,
            ROADMARK,
            SOUNDBARRIER,
            STREETLAMP,
            TRAFFICISLAND,
            TRAILER,
            TRAIN,
            TRAM,
            TREE,
            VAN,
            VEGETATION,
            WIND
        };

        RMObject(double      s,
                 double      t,
                 int         id,
                 std::string name,
                 Orientation orientation,
                 double      z_offset,
                 ObjectType  type,
                 double      length,
                 double      height,
                 double      width,
                 double      heading,
                 double      pitch,
                 double      roll,
                 double      x,
                 double      y,
                 double      z,
                 double      h)
            : s_(s),
              t_(t),
              id_(id),
              name_(name),
              orientation_(orientation),
              z_offset_(z_offset),
              type_(type),
              length_(length),
              height_(height),
              width_(width),
              heading_(heading),
              pitch_(pitch),
              roll_(roll),
              repeat_(0),
              RoadObject(x, y, z, h)
        {
        }

        ~RMObject()
        {
            for (size_t i = 0; i < outlines_.size(); i++)
                delete (outlines_[i]);
            outlines_.clear();

            for (size_t i = 0; i < repeats_.size(); i++)
                delete (repeats_[i]);
            repeats_.clear();
        }

        static std::string Type2Str(ObjectType type);
        static ObjectType  Str2Type(std::string type);

        std::string GetName() const
        {
            return name_;
        }
        std::string GetTypeStr() const
        {
            return Type2Str(type_);
        }
        ObjectType GetType() const
        {
            return type_;
        }
        int GetId() const
        {
            return id_;
        }
        double GetS() const
        {
            return s_;
        }
        void SetS(double s)
        {
            s_ = s;
        }
        double GetT() const
        {
            return t_;
        }
        double GetHOffset() const
        {
            return heading_;
        }
        double GetPitch() const
        {
            return pitch_;
        }
        double GetRoll() const
        {
            return roll_;
        }
        double GetZOffset() const
        {
            return z_offset_;
        }
        double GetHeight() const
        {
            return height_;
        }
        double GetLength() const
        {
            return length_;
        }
        double GetWidth() const
        {
            return width_;
        }
        void SetHeight(double height)
        {
            height_ = height;
        }
        void SetLength(double length)
        {
            length_ = length;
        }
        void SetWidth(double width)
        {
            width_ = width;
        }
        Orientation GetOrientation() const
        {
            return orientation_;
        }
        void AddOutline(Outline *outline)
        {
            outlines_.push_back(outline);
        }
        void SetRepeat(Repeat *repeat);
        void AddRepeat(Repeat *repeat)
        {
            repeats_.push_back(repeat);
        };
        Repeat *GetRepeat() const
        {
            return repeat_;
        }

        int GetNumberOfOutlines() const
        {
            return (int)outlines_.size();
        }
        int GetNumberOfRepeats() const
        {
            return (int)repeats_.size();
        }

        Outline *GetOutline(int i) const
        {
            return (0 <= i && i < outlines_.size()) ? outlines_[i] : 0;
        }
        Repeat *GetRepeatByIdx(int i) const
        {
            return (0 <= i && i < repeats_.size()) ? repeats_[i] : 0;
        }

    private:
        std::string            name_;
        ObjectType             type_;
        int                    id_;
        double                 s_;
        double                 t_;
        double                 z_offset_;
        Orientation            orientation_;
        double                 length_;
        double                 height_;
        double                 width_;
        double                 heading_;
        double                 pitch_;
        double                 roll_;
        std::vector<Outline *> outlines_;
        Repeat                *repeat_;
        std::vector<Repeat *>  repeats_;
    };

    enum class SpeedUnit
    {
        UNDEFINED,
        KMH,
        MS,
        MPH
    };

    class Road
    {
    public:
        enum class RoadType
        {
            ROADTYPE_UNKNOWN,
            ROADTYPE_RURAL,
            ROADTYPE_MOTORWAY,
            ROADTYPE_TOWN,
            ROADTYPE_LOWSPEED,
            ROADTYPE_PEDESTRIAN,
            ROADTYPE_BICYCLE
        };

        struct RoadTypeEntry
        {
            double    s_;
            RoadType  road_type_;
            double    speed_ = 0;  // m/s
            SpeedUnit unit_;       // Originally specified unit
        };

        enum class RoadRule
        {
            RIGHT_HAND_TRAFFIC,
            LEFT_HAND_TRAFFIC,
            ROAD_RULE_UNDEFINED
        };

        Road(int id, std::string name, RoadRule rule = RoadRule::RIGHT_HAND_TRAFFIC) : id_(id), name_(name), length_(0), junction_(-1), rule_(rule)
        {
        }
        ~Road();

        void Print() const;
        void SetId(int id)
        {
            id_ = id;
        }
        int GetId() const
        {
            return id_;
        }
        RoadRule GetRule() const
        {
            return rule_;
        }
        void SetName(std::string name)
        {
            name_ = name;
        }
        Geometry *GetGeometry(int idx) const;
        int       GetNumberOfGeometries() const
        {
            return (int)geometry_.size();
        }

        /**
        Retrieve the lanesection specified by vector element index (idx)
        useful for iterating over all available lane sections, e.g:
        for (int i = 0; i < road->GetNumberOfLaneSections(); i++)
        {
                int n_lanes = road->GetLaneSectionByIdx(i)->GetNumberOfLanes();
        ...
        @param idx index into the vector of lane sections
        */
        LaneSection *GetLaneSectionByIdx(int idx) const;

        /**
        Retrieve the lanesection index at specified s-value
        @param s distance along the road segment
        */
        int GetLaneSectionIdxByS(double s, int start_at = 0) const;

        /**
        Retrieve the lanesection at specified s-value
        @param s distance along the road segment
        */
        LaneSection *GetLaneSectionByS(double s, int start_at = 0) const
        {
            return GetLaneSectionByIdx(GetLaneSectionIdxByS(s, start_at));
        }

        /**
        Get lateral position of lane center, from road reference lane (lane id=0)
        Example: If lane id 1 is 5 m wide and lane id 2 is 4 m wide, then
        lane 1 center offset is 5/2 = 2.5 and lane 2 center offset is 5 + 4/2 = 7
        @param s distance along the road segment
        @param lane_id lane specifier, starting from center -1, -2, ... is on the right side, 1, 2... on the left
        */
        double GetCenterOffset(double s, int lane_id) const;

        int            GetLaneInfoByS(double    s,
                                      int       start_lane_link_idx,
                                      int       start_lane_id,
                                      LaneInfo &lane_info,
                                      int       laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING) const;
        int            GetConnectingLaneId(RoadLink *road_link, int fromLaneId, int connectingRoadId) const;
        double         GetLaneWidthByS(double s, int lane_id) const;
        Lane::LaneType GetLaneTypeByS(double s, int lane_id) const;
        double         GetSpeedByS(double s) const;
        bool           GetZAndPitchByS(double s, double *z, double *z_prim, double *z_primPrim, double *pitch, int *index) const;
        bool           UpdateZAndRollBySAndT(double s, double t, double *z, double *roadSuperElevationPrim, double *roll, int *index);
        int            GetNumberOfLaneSections() const
        {
            return (int)lane_section_.size();
        }
        std::string GetName() const
        {
            return name_;
        }
        void SetLength(double length)
        {
            length_ = length;
        }
        double GetLength() const
        {
            return length_;
        }
        void SetJunction(int junction)
        {
            junction_ = junction;
        }
        int GetJunction() const
        {
            return junction_;
        }
        void AddLink(RoadLink *link)
        {
            link_.push_back(link);
        }
        void AddRoadType(RoadTypeEntry *type)
        {
            type_.push_back(type);
        }
        int GetNumberOfRoadTypes() const
        {
            return (int)type_.size();
        }
        RoadTypeEntry *GetRoadType(int idx) const;
        RoadLink      *GetLink(LinkType type) const;
        void           AddLine(Line *line);
        void           AddArc(Arc *arc);
        void           AddSpiral(Spiral *spiral);
        void           AddPoly3(Poly3 *poly3);
        void           AddParamPoly3(ParamPoly3 *param_poly3);
        void           AddElevation(Elevation *elevation);
        void           AddSuperElevation(Elevation *super_elevation);
        void           AddLaneSection(LaneSection *lane_section);
        void           AddLaneOffset(LaneOffset *lane_offset);
        void           AddSignal(Signal *signal);
        void           AddObject(RMObject *object);
        Elevation     *GetElevation(int idx) const;
        Elevation     *GetSuperElevation(int idx) const;
        int            GetNumberOfSignals() const;
        Signal        *GetSignal(int idx) const;
        int            GetNumberOfObjects() const
        {
            return (int)object_.size();
        }
        RMObject *GetRoadObject(int idx) const;
        int       GetNumberOfElevations() const
        {
            return (int)elevation_profile_.size();
        }
        int GetNumberOfSuperElevations() const
        {
            return (int)super_elevation_profile_.size();
        }
        double GetLaneOffset(double s) const;
        double GetLaneOffsetPrim(double s) const;
        int    GetNumberOfLanes(double s) const;
        int    GetNumberOfDrivingLanes(double s) const;
        Lane  *GetDrivingLaneByIdx(double s, int idx) const;
        Lane  *GetDrivingLaneSideByIdx(double s, int side, int idx) const;
        Lane  *GetDrivingLaneById(double s, int idx) const;
        int    GetNumberOfDrivingLanesSide(double s, int side) const;  // side = -1 right, 1 left

        /**
                Given a lane id, get connected lane id at another longitudinal location at the same road
                taking lane sections into account
                @param lane_id Start at this lane id
                @param s_start Start from this s value
                @param s_target Stop at this s value
                @return connected lane_id if found, else 0
        */
        int GetConnectedLaneIdAtS(int lane_id, double s_start, double s_target) const;

        /**
                Check if specified road is directly connected to at specified end of current one (this)
                @param road Road to check if connected with current one
                @param contact_point If not null it will contain the contact point of specified road
                @param fromLaneId If not zero, a connection from this lane must exist on specified road
                @return true if connection exist, else false
        */
        bool IsDirectlyConnected(Road *road, LinkType link_type, ContactPointType *contact_point, int fromLaneId = 0) const;

        /**
                Check if specified road is directly connected, at least in one end of current one (this)
                @param road Road to check if connected with current one
                @param curvature Optional return parameter for curvature of checked road at connection point
                @param fromLaneId If not zero, a connection from this lane must exist on specified road
                @return true if connection exist, else false
        */
        bool IsDirectlyConnected(Road *road, double *curvature = 0, int fromLaneId = 0) const;

        /**
                Check if specified road is directly connected as successor to current one (this)
                @param road Road to check if connected with current one
                @param contact_point If not null it will contain the contact point of the successor road
                @param fromLaneId If not zero, a connection from this lane must exist on specified road
                @return true if connection exist, else false
        */
        bool IsSuccessor(Road *road, ContactPointType *contact_point = 0, int fromLaneId = 0) const;

        /**
                Check if specified road is directly connected as predecessor to current one (this)
                @param road Road to check if connected with current one
                @param contact_point If not null it will contain the contact point of the predecessor road
                @param fromLaneId If not zero, a connection from this lane must exist on specified road
                @return true if connection exist, else false
        */
        bool IsPredecessor(Road *road, ContactPointType *contact_point = 0, int fromLaneId = 0) const;

        /**
                Get width of road
                @param s Longitudinal position/distance along the road
                @param side Side of the road: -1=right, 1=left, 0=both
                @param laneTypeMask Bitmask specifying what lane types to consider - see Lane::LaneType
                @return Width (m)
        */
        double GetWidth(double s, int side, int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY) const;  // side: -1=right, 1=left, 0=both

    protected:
        int         id_;
        std::string name_;
        double      length_;
        int         junction_;
        RoadRule    rule_;

        std::vector<RoadTypeEntry *> type_;
        std::vector<RoadLink *>      link_;
        std::vector<Geometry *>      geometry_;
        std::vector<Elevation *>     elevation_profile_;
        std::vector<Elevation *>     super_elevation_profile_;
        std::vector<LaneSection *>   lane_section_;
        std::vector<LaneOffset *>    lane_offset_;
        std::vector<Signal *>        signal_;
        std::vector<RMObject *>      object_;
    };

    class LaneRoadLaneConnection
    {
    public:
        LaneRoadLaneConnection()
            : lane_id_(0),
              connecting_road_id_(-1),
              connecting_lane_id_(0),
              contact_point_(ContactPointType::CONTACT_POINT_UNDEFINED)
        {
        }
        LaneRoadLaneConnection(int lane_id, int connecting_road_id, int connecting_lane_id)
            : lane_id_(lane_id),
              connecting_road_id_(connecting_road_id),
              connecting_lane_id_(connecting_lane_id),
              contact_point_(ContactPointType::CONTACT_POINT_UNDEFINED)
        {
        }
        void SetLane(int id)
        {
            lane_id_ = id;
        }
        void SetConnectingRoad(int id)
        {
            connecting_road_id_ = id;
        }
        void SetConnectingLane(int id)
        {
            connecting_lane_id_ = id;
        }
        int GetLaneId() const
        {
            return lane_id_;
        }
        int GetConnectingRoadId() const
        {
            return connecting_road_id_;
        }
        int GetConnectinglaneId() const
        {
            return connecting_lane_id_;
        }

        ContactPointType contact_point_;

    private:
        int lane_id_;
        int connecting_road_id_;
        int connecting_lane_id_;
    };

    class JunctionLaneLink
    {
    public:
        JunctionLaneLink(int from, int to) : from_(from), to_(to)
        {
        }
        int  from_;
        int  to_;
        void Print() const
        {
            printf("JunctionLaneLink: from %d to %d\n", from_, to_);
        }
    };

    class Connection
    {
    public:
        Connection(Road *incoming_road, Road *connecting_road, ContactPointType contact_point);
        ~Connection();
        int GetNumberOfLaneLinks() const
        {
            return (int)lane_link_.size();
        }
        JunctionLaneLink *GetLaneLink(int idx) const
        {
            return lane_link_[idx];
        }
        int   GetConnectingLaneId(int incoming_lane_id) const;
        Road *GetIncomingRoad() const
        {
            return incoming_road_;
        }
        Road *GetConnectingRoad() const
        {
            return connecting_road_;
        }
        ContactPointType GetContactPoint() const
        {
            return contact_point_;
        }
        void AddJunctionLaneLink(int from, int to);
        void Print() const;

    private:
        Road                           *incoming_road_;
        Road                           *connecting_road_;
        ContactPointType                contact_point_;
        std::vector<JunctionLaneLink *> lane_link_;
    };

    typedef struct
    {
        int         signalId_;
        std::string type_;
    } Control;

    class Controller
    {
    public:
        Controller() : id_(0), name_(""), sequence_(0)
        {
        }
        Controller(int id, std::string name, int sequence = 0) : id_(id), name_(name), sequence_(sequence)
        {
        }

        void AddControl(Control ctrl)
        {
            control_.push_back(ctrl);
        }
        int GetNumberOfControls() const
        {
            return (int)control_.size();
        }
        Control *GetControl(int index)
        {
            return ((index >= 0 && index < control_.size()) ? &control_[index] : nullptr);
        }

        int GetId() const
        {
            return id_;
        }
        std::string GetName() const
        {
            return name_;
        }
        int GetSequence() const
        {
            return sequence_;
        }

    private:
        int                  id_;
        std::string          name_;
        int                  sequence_;
        std::vector<Control> control_;
    };

    typedef struct
    {
        int         id_;
        std::string type_;
        int         sequence_;
    } JunctionController;

    class Junction
    {
    public:
        typedef enum
        {
            DEFAULT,
            DIRECT,
            VIRTUAL  // not supported yet
        } JunctionType;

        typedef enum
        {
            RANDOM,
            SELECTOR_ANGLE,  // choose road which heading (relative incoming road) is closest to specified angle
        } JunctionStrategyType;

        Junction(int id, std::string name, JunctionType type) : id_(id), name_(name), type_(type)
        {
            SetGlobalId();
        }
        ~Junction();
        int GetId() const
        {
            return id_;
        }
        std::string GetName() const
        {
            return name_;
        }
        int GetNumberOfConnections() const
        {
            return (int)connection_.size();
        }
        int                    GetNumberOfRoadConnections(int roadId, int laneId) const;
        LaneRoadLaneConnection GetRoadConnectionByIdx(int roadId,
                                                      int laneId,
                                                      int idx,
                                                      int laneTypeMask = Lane::LaneType::LANE_TYPE_ANY_DRIVING) const;
        void                   AddConnection(Connection *connection)
        {
            connection_.push_back(connection);
        }
        int         GetNoConnectionsFromRoadId(int incomingRoadId) const;
        Connection *GetConnectionByIdx(int idx) const
        {
            return connection_[idx];
        }
        int  GetConnectingRoadIdFromIncomingRoadId(int incomingRoadId, int index) const;
        void Print() const;
        bool IsOsiIntersection() const;
        int  GetGlobalId() const
        {
            return global_id_;
        }
        void SetGlobalId();
        int  GetNumberOfControllers() const
        {
            return (int)controller_.size();
        }
        JunctionController *GetJunctionControllerByIdx(int index);
        void                AddController(JunctionController controller)
        {
            controller_.push_back(controller);
        }
        Road        *GetRoadAtOtherEndOfConnectingRoad(Road *connecting_road, Road *incoming_road) const;
        JunctionType GetType() const
        {
            return type_;
        }

    private:
        std::vector<Connection *>       connection_;
        std::vector<JunctionController> controller_;
        int                             id_;
        int                             global_id_;
        std::string                     name_;
        JunctionType                    type_;
    };

    typedef struct
    {
        double      a_;
        std::string axis_;
        double      b_;
        std::string ellps_;
        double      k_;
        double      k_0_;
        double      lat_0_;
        double      lon_0_;
        double      lon_wrap_;
        double      over_;
        std::string pm_;
        std::string proj_;
        std::string units_;
        std::string vunits_;
        double      x_0_;
        double      y_0_;
        std::string datum_;
        std::string geo_id_grids_;
        double      zone_;
        int         towgs84_;
    } GeoReference;

    class OpenDrive
    {
    public:
        OpenDrive() : speed_unit_(SpeedUnit::UNDEFINED), versionMajor_(0), versionMinor_(0){};
        OpenDrive(const char *filename);
        ~OpenDrive();

        /**
                Clear all allocated data and reset counters
        */
        void Clear();

        /**
                Load a road network, specified in the OpenDRIVE file format
                @param filename OpenDRIVE file
                @param replace If true any old road data will be erased, else new will be added to the old
        */
        bool LoadOpenDriveFile(const char *filename, bool replace = true);

        /**
                Initialize the global ids for lanes
        */
        void InitGlobalLaneIds();

        /**
                Get the filename of currently loaded OpenDRIVE file
        */
        std::string GetOpenDriveFilename() const
        {
            return odr_filename_;
        }

        /**
                Setting information based on the OSI standards for OpenDrive elements
        */
        bool SetRoadOSI();
        bool CheckLaneOSIRequirement(std::vector<double> x0, std::vector<double> y0, std::vector<double> x1, std::vector<double> y1) const;
        void SetLaneOSIPoints();
        void SetRoadMarkOSIPoints();

        /**
                Checks all lanes - if a lane has RoadMarks it does nothing. If a lane does not have roadmarks
                then it creates a LaneBoundary following the lane border (left border for left lanes, right border for right lanes)
        */
        void SetLaneBoundaryPoints();

        /**
                Retrieve a road segment specified by road ID
                @param id road ID as specified in the OpenDRIVE file
        */
        Road *GetRoadById(int id) const;

        /**
                Retrieve a road segment specified by road vector element index
                useful for iterating over all available road segments, e.g:
                for (int i = 0; i < GetNumOfRoads(); i++)
                {
                        int n_lanes = GetRoadyIdx(i)->GetNumberOfLanes();
                ...
                @param idx index into the vector of roads
        */
        Road     *GetRoadByIdx(int idx) const;
        Geometry *GetGeometryByIdx(int road_idx, int geom_idx) const;
        int       GetTrackIdxById(int id) const;
        int       GetTrackIdByIdx(int idx) const;
        int       GetNumOfRoads() const
        {
            return (int)road_.size();
        }
        Junction *GetJunctionById(int id) const;
        Junction *GetJunctionByIdx(int idx) const;

        int GetNumOfJunctions() const
        {
            return (int)junction_.size();
        }

        bool IsIndirectlyConnected(int road1_id, int road2_id, int *&connecting_road_id, int *&connecting_lane_id, int lane1_id = 0, int lane2_id = 0)
            const;

        /**
                Add any missing connections so that road connectivity is two-ways
                Look at all road connections, and make sure they are defined both ways
                @param idx index into the vector of roads
                @return number of added connections
        */
        int                CheckConnections();
        int                CheckLink(Road *road, RoadLink *link, ContactPointType expected_contact_point_type);
        int                CheckConnectedRoad(Road *road, RoadLink *link, ContactPointType expected_contact_point_type, RoadLink *link2);
        int                CheckJunctionConnection(Junction *junction, Connection *connection);
        static std::string ContactPointType2Str(ContactPointType type);
        static std::string ElementType2Str(RoadLink::ElementType type);
        static std::string LinkType2Str(LinkType link_type);

        int GetNumberOfControllers() const
        {
            return (int)controller_.size();
        }
        Controller *GetControllerByIdx(int index);
        Controller *GetControllerById(int id);
        void        AddController(Controller controller)
        {
            controller_.push_back(controller);
        }

        GeoReference *GetGeoReference();
        std::string   GetGeoReferenceAsString() const;
        void          ParseGeoLocalization(const std::string &geoLocalization);

        bool LoadSignalsByCountry(const std::string &country);

        void SetSpeedUnit(SpeedUnit unit)
        {
            speed_unit_ = unit;
        }
        /**
                Get first specified speed unit (in road type elements). MS is default.
                @return 0=Undefined, 1=km/h 2=m/s, 3=mph
        */
        SpeedUnit GetSpeedUnit() const
        {
            return speed_unit_;
        }

        int GetVersionMajor() const
        {
            return versionMajor_;
        }
        int GetVersionMinor() const
        {
            return versionMinor_;
        }

        void Print() const;

    private:
        pugi::xml_node                     root_node_;
        std::vector<Road *>                road_;
        std::vector<Junction *>            junction_;
        std::vector<Controller>            controller_;
        GeoReference                       geo_ref_;
        std::string                        odr_filename_;
        std::map<std::string, std::string> signals_types_;
        SpeedUnit                          speed_unit_;  // First specified speed unit. MS is default. Undefined if no speed entries.
        int                                versionMajor_;
        int                                versionMinor_;
    };

    typedef struct
    {
        double pos[3];       // position, in global coordinate system
        double heading;      // road heading at steering target point
        double pitch;        // road pitch (inclination) at steering target point
        double roll;         // road roll (camber) at steering target point
        double width;        // lane width
        double curvature;    // road curvature at steering target point
        double speed_limit;  // speed limit given by OpenDRIVE type entry
        int    roadId;       // road ID
        int    junctionId;   // junction ID (-1 if not in a junction)
        int    laneId;       // lane ID
        double laneOffset;   // lane offset (lateral distance from lane center)
        double s;            // s (longitudinal distance along reference line)
        double t;            // t (lateral distance from reference line)
    } RoadLaneInfo;

    typedef struct
    {
        RoadLaneInfo road_lane_info;   // Road info at probe target position
        double       relative_pos[3];  // probe target position relative vehicle (pivot position object) coordinate system
        double       relative_h;       // heading angle to probe target from and relatove to vehicle (pivot position)
    } RoadProbeInfo;

    typedef struct
    {
        double ds;        // delta s (longitudinal distance)
        double dt;        // delta t (lateral distance)
        int    dLaneId;   // delta laneId (increasing left and decreasing to the right)
        double dx;        // delta x (world coordinate system)
        double dy;        // delta y (world coordinate system)
        bool   dOppLane;  // true if the two position objects are in opposite sides of reference lane
    } PositionDiff;

    enum class CoordinateSystem
    {
        CS_UNDEFINED,
        CS_ENTITY,
        CS_LANE,
        CS_ROAD,
        CS_TRAJECTORY
    };

    enum class RelativeDistanceType
    {
        REL_DIST_UNDEFINED,
        REL_DIST_LATERAL,
        REL_DIST_LONGITUDINAL,
        REL_DIST_CARTESIAN,
        REL_DIST_EUCLIDIAN
    };

    // Forward declarations
    class Route;
    class RMTrajectory;

    typedef enum
    {
        INTERPOLATE_HEADING = 0x1,
        INTERPOLATE_PITCH   = 0x2,
        INTERPOLATE_ROLL    = 0x4
    } InterpolationComponent;

    struct TrajVertex
    {
        double s           = 0;
        double x           = 0;
        double y           = 0;
        double z           = 0;
        double h           = 0;
        double pitch       = 0;
        double r           = 0;
        int    road_id     = -1;  // -1 indicates no valid road position. Use X, Y instead.
        double time        = 0;
        double speed       = 0;  // speed at vertex point/start of segment
        double acc         = 0;  // acceleration along the segment
        double param       = 0;
        int    pos_mode    = 0;  // resolved alignment bitmask after calculation, see Position::PosMode enum
        int    interpolate = 0;  // interpolation bitmask, see InterpolateComponent enum
    };

    class Position
    {
    public:
        typedef enum
        {
            SHORTEST,
            FASTEST,
            MIN_INTERSECTIONS
        } RouteStrategy;

        enum class PositionType
        {
            NORMAL,
            ROUTE,
            RELATIVE_OBJECT,
            RELATIVE_WORLD,
            RELATIVE_LANE,
            RELATIVE_ROAD
        };

        enum class OrientationType
        {
            ORIENTATION_RELATIVE,
            ORIENTATION_ABSOLUTE
        };

        enum class LookAheadMode
        {
            LOOKAHEADMODE_AT_LANE_CENTER,
            LOOKAHEADMODE_AT_ROAD_CENTER,
            LOOKAHEADMODE_AT_CURRENT_LATERAL_OFFSET,
        };

        enum class ReturnCode
        {
            ERROR_OFF_ROAD       = -5,
            ERROR_NOT_ON_ROUTE   = -4,
            ERROR_END_OF_ROUTE   = -3,
            ERROR_END_OF_ROAD    = -2,
            ERROR_GENERIC        = -1,
            OK                   = 0,
            ENTERED_NEW_ROAD     = 1,  // position moved into a new road segment
            MADE_JUNCTION_CHOICE = 2,  // position moved into a junction and made a choice
        };

        enum class UpdateTrackPosMode
        {
            UPDATE_NOT_XYZH,
            UPDATE_XYZ,
            UPDATE_XYZH
        };

        enum class OrientationSetMask
        {
            H = 1 << 0,
            P = 1 << 1,
            R = 1 << 2
        };

        enum class PositionStatusMode
        {
            POS_STATUS_END_OF_ROAD  = (1 << 0),
            POS_STATUS_END_OF_ROUTE = (1 << 1)
        };

        // Modes for interpret Z, Head, Pitch, Roll coordinate value as absolute or relative
        // grouped as bitmask: 0000 => skip/use current, 0001=DEFAULT, 0011=ABS, 0111=REL
        // example: Relative Z, Absolute H, Default R, Current P = Z_REL | H_ABS | R_DEF = 4151 = 0001 0000 0011 0111
        typedef enum
        {
            Z_SET  = 1,   // 0001
            Z_DEF  = 1,   // 0001
            Z_ABS  = 3,   // 0011
            Z_REL  = 7,   // 0111
            Z_MASK = 15,  // 1111
            H_SET  = Z_SET << 4,
            H_DEF  = Z_DEF << 4,
            H_ABS  = Z_ABS << 4,
            H_REL  = Z_REL << 4,
            H_MASK = Z_MASK << 4,
            P_SET  = Z_SET << 8,
            P_DEF  = Z_DEF << 8,
            P_ABS  = Z_ABS << 8,
            P_REL  = Z_REL << 8,
            P_MASK = Z_MASK << 8,
            R_SET  = Z_SET << 12,
            R_DEF  = Z_DEF << 12,
            R_ABS  = Z_ABS << 12,
            R_REL  = Z_REL << 12,
            R_MASK = Z_MASK << 12,
        } PosMode;

        // Types of position modes
        enum class PosModeType
        {
            SET    = 0,  // Used by explicit set functions
            UPDATE = 1,  // Used by controllers updating the position
            INIT   = 2   // Indicate mode at initialization, i.e. what components were set (ABS) and not (REL)
        };

        bool CheckBitsEqual(int input, int mask, int bits) const
        {
            return (input & mask) == (bits & mask);
        }

        enum class DirectionMode
        {
            ALONG_S,
            ALONG_LANE
        };

        explicit Position();
        explicit Position(int track_id, double s, double t);
        explicit Position(int track_id, int lane_id, double s, double offset);
        explicit Position(double x, double y, double z, double h, double p, double r);
        explicit Position(double x, double y, double z, double h, double p, double r, bool calculateTrackPosition);
        ~Position();

        void              Init();
        static bool       LoadOpenDrive(const char *filename);
        static bool       LoadOpenDrive(OpenDrive *odr);
        static OpenDrive *GetOpenDrive();
        int               GotoClosestDrivingLaneAtCurrentPosition();

        /**
        Specify position by track coordinate (road_id, s, t) using current UPDATE mode
        @param track_id Id of the road (track)
        @param s Distance to the position along and from the start of the road (track)
        @param updateXY update world coordinates x, y... as well - or not
        @return Non zero return value indicates error of some kind
        */
        ReturnCode SetTrackPos(int track_id, double s, double t, bool UpdateXY = true);

        /**
        Specify position by lane coordinate (road_id, s, t) with specified mode
        @param track_id Id of the road (track)
        @param s Distance to the position along and from the start of the road (track)
        @param mode Bitmask combining values from roadmanager::PosMode enum
        example: To set relative z and absolute roll: (Z_REL | R_ABS) or (7 | 12288) = (7 + 12288) = 12295
        @param updateXY update world coordinates x, y... as well - or not
        @return Non zero return value indicates error of some kind
        */
        ReturnCode SetTrackPosMode(int track_id, double s, double t, int mode, bool UpdateXY = true);
        void       ForceLaneId(int lane_id);

        /**
        Specify position by lane coordinate (road_id, lane_id, s, lane offset) using current UPDATE mode
        @param track_id Id of the road (track)
        @param lane_id Id of the lane
        @param s Distance to the position along and from the start of the road (track)
        @param offset Lateral distance from lane center
        @param lane_section_idx Optional index of lane section to start search from
        @return Non zero return value indicates error of some kind
        */
        ReturnCode SetLanePos(int track_id, int lane_id, double s, double offset, int lane_section_idx = -1);

        /**
        Specify position by lane coordinate (road_id, lane_id, s, lane offset) with specified mode
        @param track_id Id of the road (track)
        @param lane_id Id of the lane
        @param s Distance to the position along and from the start of the road (track)
        @param offset Lateral distance from lane center
        @param mode Bitmask combining values from roadmanager::PosMode enum
        example: To set relative z and absolute roll: (Z_REL | R_ABS) or (7 | 12288) = (7 + 12288) = 12295
        @param lane_section_idx Optional index of lane section to start search from
        @return Non zero return value indicates error of some kind
        */
        ReturnCode SetLanePosMode(int track_id, int lane_id, double s, double offset, int mode, int lane_section_idx = -1);

        void SetLaneBoundaryPos(int track_id, int lane_id, double s, double offset, int lane_section_idx = -1);
        void SetRoadMarkPos(int    track_id,
                            int    lane_id,
                            int    roadmark_idx,
                            int    roadmarktype_idx,
                            int    roadmarkline_idx,
                            double s,
                            double offset,
                            int    lane_section_idx = -1);

        /**
        Specify position by cartesian x, y, z and heading, pitch, roll using current SET mode
        @param x x
        @param y y
        @param z z (if std::nan("") it will be ignored/no change)
        @param h heading
        @param p pitch (if std::nan("") it will be ignored/no change)
        @param r roll (if std::nan("") it will be ignored/no change)
        @param updateTrackPos True: road position will be calculated False: don't update road position
        @return Non zero return value indicates error of some kind
        */
        int SetInertiaPos(double x, double y, double z, double h, double p, double r, bool updateTrackPos = true);

        /**
        Specify position by cartesian x, y, z and heading, pitch, roll using specified mode
        @param x x
        @param y y
        @param z z (if std::nan("") it will be aligned to road)
        @param h heading
        @param p pitch (if std::nan("") it will be ignored/no change)
        @param r roll (if std::nan("") it will be ignored/no change)
        @param mode Bitmask combining values from roadmanager::PosMode enum
        example: To set relative z and absolute roll: (Z_REL | R_ABS) or (7 | 12288) = (7 + 12288) = 12295
        @param updateTrackPos True: road position will be calculated False: don't update road position
        @return Non zero return value indicates error of some kind
        */
        int SetInertiaPosMode(double x, double y, double z, double h, double p, double r, int mode, bool updateTrackPos = true);

        /**
        Specify position by cartesian x, y and heading using current SET mode for heading and UPDATE mode for pitch and roll
        @param x x
        @param y y
        @param h heading
        @param updateTrackPos True: road position will be calculated False: don't update road position
        @return Non zero return value indicates error of some kind
        */
        int SetInertiaPos(double x, double y, double h, bool updateTrackPos = true);

        /**
        Specify position by cartesian x, y and heading. Z, pitch and roll will be set to zero.
        Heading, Z, pitch and roll will be aligned to road according to specified mode.
        @param x x
        @param y y
        @param h heading
        @param mode Bitmask combining values from roadmanager::PosMode enum
        example: To set relative z and absolute roll: (Z_REL | R_ABS) or (7 | 12288) = (7 + 12288) = 12295
        @param updateTrackPos True: road position will be calculated False: don't update road position
        @return Non zero return value indicates error of some kind
        */
        int  SetInertiaPosMode(double x, double y, double h, int mode, bool updateTrackPos = true);
        void SetHeading(double heading);
        void SetHeadingRelative(double heading);
        void SetHeadingRelativeRoadDirection(double heading);
        void SetRoll(double roll);
        void SetRollRelative(double roll);
        void SetPitch(double roll);
        void SetPitchRelative(double pitch);
        void SetZ(double z);
        void SetZRelative(double z);

        /**
        Call this to resolve orientation alignment wrt road
        @param mode Bitmask combining values from roadmanager::PosMode enum
        example: To set relative z and absolute roll: (Z_REL | R_ABS) or (7 | 12288) = (7 + 12288) = 12295
        */
        void EvaluateZHPR(int align_mode_mask);

        /**
        Call this to resolve orientation alignment wrt road, using current SET mode
        */
        void EvaluateZHPR();

        /**
        Specify position by cartesian coordinate (x, y, z, h)
        @param x X-coordinate
        @param y Y-coordinate
        @param z Z-coordinate
        @param h Heading
        @param connectedOnly If true only roads that can be reached from current position will be considered, if false all roads will be considered
        @param roadId If != -1 only this road will be considered else all roads will be searched
        @param check_overlapping_roads If true all roads ovlerapping the position will be registered (with some performance penalty)
        @return Non zero return value indicates error of some kind
        */
        ReturnCode XYZ2TrackPos(double x, double y, double z, bool connectedOnly = false, int roadId = -1, bool check_overlapping_roads = false);

        int TeleportTo(Position *pos);

        ReturnCode MoveToConnectingRoad(RoadLink *road_link, ContactPointType &contact_point_type, double junctionSelectorAngle = -1.0);

        void SetRelativePosition(Position *rel_pos, PositionType type)
        {
            rel_pos_ = rel_pos;
            type_    = type;
        }

        Position *GetRelativePosition() const
        {
            return rel_pos_;
        }

        void ReleaseRelation();

        int                       SetRoute(Route *route);
        int                       CalcRoutePosition();
        const roadmanager::Route *GetRoute() const
        {
            return route_;
        }
        Route *GetRoute()
        {
            return route_;
        }
        RMTrajectory *GetTrajectory()
        {
            return trajectory_;
        }

        void SetTrajectory(RMTrajectory *trajectory);

        /**
        Set the current position along the route.
        @param position A regular position created with road, lane or world coordinates
        @return Non zero return value indicates error of some kind
        */
        int SetRoutePosition(Position *position);

        /**
        Retrieve the S-value of the current route position. Note: This is the S along the
        complete route, not the actual individual roads.
        */
        double GetRouteS() const;
        /**
        Move current position forward, or backwards, ds meters along the route
        @param ds Distance to move, negative will move backwards
        @param actualDistance Distance considering lateral offset and curvature (true/default) or along centerline (false)
        @return Non zero return value indicates error of some kind, most likely End Of Route
        */
        ReturnCode MoveRouteDS(double ds, bool actualDistance = true);

        /**
        Move current position to specified S-value along the route
        @param route_s Distance to move, negative will move backwards
        @return Non zero return value indicates error of some kind, most likely End Of Route
        */
        ReturnCode SetRouteS(double route_s);

        /**
        Move current position along the route
        @param ds Distance to move, negative will move backwards
        @return Non zero return value indicates error of some kind
        */
        int SetRouteLanePosition(Route *route, double path_s, int lane_id, double lane_offset);

        /**
        Move current position forward, or backwards, ds meters along the trajectory
        @param ds Distance to move, negative will move backwards
        @return Non zero return value indicates error of some kind
        */
        int MoveTrajectoryDS(double ds);

        /**
        Move current position to specified S-value along the trajectory
        @param trajectory_s Distance from start of the trajectory
        @return Non zero return value indicates error of some kind
        */
        int SetTrajectoryS(double trajectory_s);

        int SetTrajectoryPosByTime(double time);

        /**
        Retrieve the S-value of the current trajectory position
        */
        double GetTrajectoryS() const
        {
            return s_trajectory_;
        }

        /**
        Move current position to specified T-value along the trajectory
        @param trajectory_t Lateral distance from trajectory at current s-value
        @return Non zero return value indicates error of some kind
        */
        void SetTrajectoryT(double trajectory_t)
        {
            t_trajectory_ = trajectory_t;
        }

        /**
        Retrieve the T-value of the current trajectory position
        */
        double GetTrajectoryT() const
        {
            return t_trajectory_;
        }

        /**
        Straight (not route) distance between the current position and the one specified in argument
        @param target_position The position to measure distance from current position.
        @param x (meter). X component of the relative distance.
        @param y (meter). Y component of the relative distance.
        @return distance (meter). Negative if the specified position is behind the current one.
        */
        double getRelativeDistance(double targetX, double targetY, double &x, double &y) const;

        /**
        Find out the difference between two position objects, in effect subtracting the values
        It can be used to calculate the distance from current position to another one (pos_b)
        @param pos_b The position from which to subtract the current position (this position object)
        @param diff Return argument, a struct that will contain the result. dx and dy will only be set when no route found.
        @param bothDirections Set to true in order to search also backwards from object
        @param maxDist Don't look further than this
        @return true if position found and parameter values are valid, else false
        */
        bool Delta(Position *pos_b, PositionDiff &diff, bool bothDirections = true, double maxDist = LARGE_NUMBER) const;

        /**
        Find out the distance, on specified system and type, between two position objects
        @param pos_b The position from which to subtract the current position (this position object)
        @param dist Distance (output parameter)
        @return 0 if position found and parameter values are valid, else -1
        */
        int Distance(Position *pos_b, CoordinateSystem cs, RelativeDistanceType relDistType, double &dist, double maxDist = LARGE_NUMBER) const;

        /**
        Find out the distance, on specified system and type, to a world x, y position
        @param x X coordinate of position from which to subtract the current position (this position object)
        @param y Y coordinate of position from which to subtract the current position (this position object)
        @param dist Distance (output parameter)
        @return 0 if position found and parameter values are valid, else -1
        */
        int Distance(double x, double y, CoordinateSystem cs, RelativeDistanceType relDistType, double &dist, double maxDist = LARGE_NUMBER) const;

        /**
        Is the current position ahead of the one specified in argument
        This method is more efficient than getRelativeDistance
        @param target_position The position to compare the current to.
        @return true of false
        */
        bool IsAheadOf(Position target_position) const;

        /**
        Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
        @param lookahead_distance The distance, along the road, to the point
        @param data Struct to fill in calculated values, see typdef for details
        @param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode
        enum
        @return 0 if successful, other codes see Position::ErrorCode
        */
        ReturnCode GetProbeInfo(double lookahead_distance, RoadProbeInfo *data, LookAheadMode lookAheadMode) const;

        /**
        Get information suitable for driver modeling of a point at a specified distance from object along the road ahead
        @param target_pos The target position
        @param data Struct to fill in calculated values, see typdef for details
        @return 0 if successful, other codes see Position::ErrorCode
        */
        ReturnCode GetProbeInfo(Position *target_pos, RoadProbeInfo *data) const;

        /**
        Get information of current lane at a specified distance from object along the road ahead
        @param lookahead_distance The distance, along the road, to the point
        @param data Struct to fill in calculated values, see typdef for details
        @param lookAheadMode Measurement strategy: Along reference lane, lane center or current lane offset. See roadmanager::Position::LookAheadMode
        enum
        @return 0 if successful, -1 if not
        */
        int GetRoadLaneInfo(double lookahead_distance, RoadLaneInfo *data, LookAheadMode lookAheadMode) const;
        int GetRoadLaneInfo(RoadLaneInfo *data) const;

        /**
        Get information of current lane at a specified distance from object along the road ahead
        @param lookahead_distance The distance, along the road, to the point
        @param data Struct to fill in calculated values, see typdef for details
        @return 0 if successful, -1 if not
        */
        int CalcProbeTarget(Position *target, RoadProbeInfo *data) const;

        double DistanceToDS(double ds);

        /**
        Move position along the road network, forward or backward, from the current position
        It will automatically follow connecting lanes between connected roads
        If reaching a junction, choose way according to specified junctionSelectorAngle
        @param ds distance to move from current position
        @param dLaneOffset delta lane offset (adding to current position lane offset)
        @param junctionSelectorAngle Desired direction [0:2pi] from incoming road direction (angle = 0), set -1 to randomize
        @param actualDistance Distance considering lateral offset and curvature (true/default) or along centerline (false)
        @return >= 0 on success, < 0 on error. For all codes see esmini roadmanager::Position::enum class ReturnCode
        */
        ReturnCode MoveAlongS(double ds, double dLaneOffset, double junctionSelectorAngle, bool actualDistance = true);

        /**
        Move position along the road network, forward or backward, from the current position
        It will automatically follow connecting lanes between connected roads
        If multiple options (only possible in junctions) it will choose randomly
        @param ds distance to move from current position
        @return 0 if successful, other codes see Position::ReturnCode
        */
        ReturnCode MoveAlongS(double ds, bool actualDistance = true)
        {
            return MoveAlongS(ds, 0.0, -1.0, actualDistance);
        }

        /**
        Retrieve the track/road ID from the position object
        @return track/road ID
        */
        int GetTrackId() const;

        /**
        Retrieve the junction ID from the position object
        @return junction ID, -1 if not in a junction
        */
        int GetJunctionId() const;

        /**
        Retrieve the lane ID from the position object
        @return lane ID
        */
        int GetLaneId() const;
        /**
        Retrieve the global lane ID from the position object
        @return lane ID
        */
        int GetLaneGlobalId() const;

        /**
        Retrieve a road segment specified by road ID
        @param id road ID as specified in the OpenDRIVE file
        */
        Road *GetRoadById(int id) const
        {
            return GetOpenDrive()->GetRoadById(id);
        }

        /**
        Retrieve the s value (distance along the road segment)
        */
        double GetS() const;

        /**
        Retrieve the t value (lateral distance from reference lanem (id=0))
        */
        double GetT() const;

        /**
        Retrieve the offset from current lane
        */
        double GetOffset() const;

        /**
        Retrieve the world coordinate X-value
        */
        double GetX() const;

        /**
        Retrieve the world coordinate Y-value
        */
        double GetY() const;

        /**
        Retrieve the world coordinate Z-value
        */
        double GetZ() const;

        /**
        Retrieve the road Z-value
        */
        double GetZRoad() const
        {
            return z_road_;
        }

        /**
        Retrieve the road slope (vertical inclination)
        */
        double GetZRoadPrim() const
        {
            return z_roadPrim_;
        }

        /**
        Retrieve the road slope rate of change (vertical bend)
        */
        double GetZRoadPrimPrim() const
        {
            return z_roadPrimPrim_;
        }

        /**
        Retrieve the road rate of change of the road lateral inclination
        */
        double GetRoadSuperElevationPrim() const
        {
            return roadSuperElevationPrim_;
        }

        /**
        Retrieve the world coordinate heading angle (radians)
        */
        double GetH() const;

        /**
        Retrieve the road heading angle (radians)
        */
        double GetHRoad() const
        {
            return h_road_;
        }

        /**
        Retrieve the driving direction considering lane ID and rult (lef or right hand traffic)
        Will be either 1 (road direction) or -1 (opposite road direction)
        */
        int GetDrivingDirectionRelativeRoad() const;

        /**
        Retrieve the road heading angle (radians) relative driving direction (lane sign considered)
        */
        double GetHRoadInDrivingDirection() const;

        /**
        Retrieve the heading angle (radians) relative driving direction (lane sign considered)
        */
        double GetHRelativeDrivingDirection() const;

        /**
        Retrieve the relative heading angle (radians)
        */
        double GetHRelative() const;

        /**
        Retrieve the world coordinate pitch angle (radians)
        */
        double GetP() const;

        /**
        Retrieve the road pitch value
        */
        double GetPRoad() const
        {
            return p_road_;
        }

        /**
        Retrieve the relative pitch angle (radians)
        */
        double GetPRelative() const;

        /**
        Retrieve the world coordinate roll angle (radians)
        */
        double GetR() const;

        /**
        Retrieve the road roll value
        */
        double GetRRoad() const
        {
            return r_road_;
        }

        /**
        Retrieve the relative roll angle (radians)
        */
        double GetRRelative() const;

        /**
        Retrieve the road pitch value, driving direction considered
        */
        double GetPRoadInDrivingDirection() const;

        /**
        Retrieve the road curvature at current position
        */
        double GetCurvature() const;

        /**
        Retrieve the speed limit at current position
        */
        double GetSpeedLimit() const;

        /**
        Retrieve the road heading at current position
        */
        double GetRoadH() const;

        /**
        Retrieve the road heading/direction at current position, and in the direction given by current lane
        */
        double GetDrivingDirection() const;

        PositionType GetType()
        {
            return type_;
        }

        void SetTrackId(int trackId)
        {
            track_id_ = trackId;
        }
        void SetLaneId(int laneId)
        {
            lane_id_ = laneId;
        }
        void SetS(double s)
        {
            s_ = s;
        }
        void SetOffset(double offset)
        {
            offset_ = offset;
        }
        void SetT(double t)
        {
            t_ = t;
        }
        void SetX(double x)
        {
            x_ = x;
        }
        void SetY(double y)
        {
            y_ = y;
        }
        void SetH(double h)
        {
            h_ = h;
        }
        void SetP(double p)
        {
            p_ = p;
        }
        void SetR(double r)
        {
            r_ = r;
        }
        void SetVel(double x_vel, double y_vel, double z_vel)
        {
            velX_ = x_vel, velY_ = y_vel, velZ_ = z_vel;
        }
        void SetAcc(double x_acc, double y_acc, double z_acc)
        {
            accX_ = x_acc, accY_ = y_acc, accZ_ = z_acc;
        }
        void SetAngularVel(double h_vel, double p_vel, double r_vel)
        {
            h_rate_ = h_vel, p_rate_ = p_vel, r_rate_ = r_vel;
        }
        void SetAngularAcc(double h_acc, double p_acc, double r_acc)
        {
            h_acc_ = h_acc, p_acc_ = p_acc, r_acc_ = r_acc;
        }
        double GetVelX() const
        {
            return velX_;
        }
        double GetVelY() const
        {
            return velY_;
        }
        double GetVelZ() const
        {
            return velZ_;
        }

        /**
        Get lateral component of velocity in vehicle local coordinate system
        */
        double GetVelLat() const;

        /**
        Get longitudinal component of velocity in vehicle local coordinate system
        */
        double GetVelLong() const;

        /**
        Get lateral and longitudinal component of velocity in vehicle local coordinate system
        This is slightly more effecient than calling GetVelLat and GetVelLong separately
        @param vlat reference parameter returning lateral velocity
        @param vlong reference parameter returning longitudinal velocity
        @return -
        */
        void GetVelLatLong(double &vlat, double &vlong) const;

        /**
        Get lateral component of acceleration in vehicle local coordinate system
        */
        double GetAccLat() const;

        /**
        Get longitudinal component of acceleration in vehicle local coordinate system
        */
        double GetAccLong() const;

        /**
        Get lateral and longitudinal component of acceleration in vehicle local coordinate system
        This is slightly more effecient than calling GetAccLat and GetAccLong separately
        @param alat reference parameter returning lateral acceleration
        @param along reference parameter returning longitudinal acceleration
        @return -
        */
        void GetAccLatLong(double &alat, double &along) const;

        /**
        Get lateral component of velocity in road coordinate system
        */
        double GetVelT() const;

        /**
        Get longitudinal component of velocity in road coordinate system
        */
        double GetVelS() const;

        /**
        Get lateral and longitudinal component of velocity in road coordinate system
        This is slightly more effecient than calling GetVelT and GetVelS separately
        @param vt reference parameter returning lateral velocity
        @param vs reference parameter returning longitudinal velocity
        @return -
        */
        void GetVelTS(double &vt, double &vs) const;

        /**
        Get lateral component of acceleration in road coordinate system
        */
        double GetAccT() const;

        /**
        Get longitudinal component of acceleration in road coordinate system
        */
        double GetAccS() const;

        /**
        Get magnitude of acceleration. Sign indicating direction, accelerating (+) or decelerating (-)
        */
        double GetAcc() const;

        /**
        Get lateral and longitudinal component of acceleration in road coordinate system
        This is slightly more effecient than calling GetAccT and GetAccS separately
        @param at reference parameter returning lateral acceleration
        @param as reference parameter returning longitudinal acceleration
        @return -
        */
        void GetAccTS(double &at, double &as) const;

        double GetAccX() const
        {
            return accX_;
        }
        double GetAccY() const
        {
            return accY_;
        }
        double GetAccZ() const
        {
            return accZ_;
        }
        double GetHRate() const
        {
            return h_rate_;
        }
        double GetPRate() const
        {
            return p_rate_;
        }
        double GetRRate() const
        {
            return r_rate_;
        }
        double GetHAcc() const
        {
            return h_acc_;
        }
        double GetPAcc() const
        {
            return p_acc_;
        }
        double GetRAcc() const
        {
            return r_acc_;
        }

        int GetStatusBitMask() const
        {
            return status_;
        }

        void SetOrientationType(OrientationType type)
        {
            orientation_type_ = type;
        }

        OrientationType GetOrientationType() const
        {
            return orientation_type_;
        }

        /**
        Set default road alignment for object
        @param type 0=Set (for all explicit set-functions), 1=Update (when object is updated by any controller)
        */
        void SetModeDefault(PosModeType type)
        {
            SetMode(type, GetModeDefault(type));
        }

        /**
        Get default road alignment for object
        @param type 0=Set (for all explicit set-functions), 1=Update (when object is updated by any controller)
        */
        static const int GetModeDefault(PosModeType type)
        {
            if (type == PosModeType::SET)
            {
                return PosMode::Z_REL | PosMode::H_ABS | PosMode::P_REL | PosMode::R_REL;
            }
            else if (type == PosModeType::UPDATE)
            {
                return PosMode::Z_REL | PosMode::H_REL | PosMode::P_REL | PosMode::R_REL;
            }
            else if (type == PosModeType::INIT)
            {
                return PosMode::Z_REL | PosMode::H_ABS | PosMode::P_REL | PosMode::R_REL;
            }
            else
            {
                LOG("Unexpected position mode type: %d", type);
                return 0;
            }
        }

        /**
        Specify if and how position object will align to the road. This version
        sets same mode for all components: Heading, Pitch, Roll and Z (elevation)
        @param id Id of the object
        @param mode Bitmask combining values from roadmanager::PosMode enum
        example: To set relative z and absolute roll: (Z_REL | R_ABS) or (7 | 12288) = (7 + 12288) = 12295
        @param type 0=Set (for all explicit set-functions), 1=Update (when object is updated by any controller)
        */
        void SetMode(PosModeType type, int mode);

        int GetMode(PosModeType type);

        /**
        Specify which lane types the position object snaps to (is aware of)
        @param laneTypes A combination (bitmask) of lane types
        @return -
        */
        void SetSnapLaneTypes(int laneTypeMask)
        {
            snapToLaneTypes_ = laneTypeMask;
        }

        int GetSnapLaneTypes() const
        {
            return snapToLaneTypes_;
        }

        void CopyRMPos(Position *from);

        void PrintTrackPos() const;
        void PrintLanePos() const;
        void PrintInertialPos() const;

        void Print() const;
        void PrintXY() const;

        bool IsOffRoad() const;
        bool IsInJunction() const;

        /**
        Get the number of roads overlapping the position
        @return Number of roads overlapping the position
        */
        int GetNumberOfRoadsOverlapping();

        /**
        Get the id of an overlapping road
        @parameter index Index of the total returned by GetNumberOfRoadsOverlapping()
        @return Id of specified overlapping road
        */
        int GetOverlappingRoadId(int index) const;

        void ReplaceObjectRefs(Position *pos1, Position *pos2)
        {
            if (rel_pos_ == pos1)
            {
                rel_pos_ = pos2;
            }
        }

        /**
                Controls whether to keep lane ID regardless of lateral position or snap to closest lane (default)
                @parameter mode True=keep lane False=Snap to closest (default)
        */
        void SetLockOnLane(bool mode)
        {
            lockOnLane_ = mode;
        }

        RouteStrategy GetRouteStrategy() const
        {
            return routeStrategy_;
        }
        void SetRouteStrategy(RouteStrategy rs)
        {
            routeStrategy_ = rs;
        }

        void SetDirectionMode(DirectionMode direction_mode)
        {
            direction_mode_ = direction_mode;
        }

        DirectionMode GetDirectionMode() const
        {
            return direction_mode_;
        }

    protected:
        void       Track2Lane();
        ReturnCode Track2XYZ();
        void       Lane2Track();
        void       RoadMark2Track();
        /**
        Set position to the border of lane (right border for right lanes, left border for left lanes)
        */
        void       LaneBoundary2Track();
        void       XYZ2Track();
        ReturnCode SetLongitudinalTrackPos(int track_id, double s);
        bool       EvaluateRoadZHPR();

        /**
        Update trajectory position
        @param v Trajectory vertex
        @return Non zero return value indicates error of some kind
        */
        int UpdateTrajectoryPos(TrajVertex v);

        // Control lane belonging
        bool lockOnLane_;  // if true then keep logical lane regardless of lateral position, default false

        // route reference
        Route *route_;  // if pointer set, the position corresponds to a point along (s) the route

        // route reference
        RMTrajectory *trajectory_;  // if pointer set, the position corresponds to a point along (s) the trajectory

        // track reference
        int    track_id_;
        double s_;             // longitudinal point/distance along the track
        double t_;             // lateral position relative reference line (geometry)
        int    lane_id_;       // lane reference
        double offset_;        // lateral position relative lane given by lane_id
        double h_road_;        // heading of the road
        double h_offset_;      // local heading offset given by lane width and offset
        double h_relative_;    // heading relative to the road (h_ = h_road_ + h_relative_)
        double z_relative_;    // z relative to the road
        double s_trajectory_;  // longitudinal point/distance along the trajectory
        double t_trajectory_;  // longitudinal point/distance along the trajectory
        double curvature_;
        double p_relative_;   // pitch relative to the road (h_ = h_road_ + h_relative_)
        double r_relative_;   // roll relative to the road (h_ = h_road_ + h_relative_)
        int    mode_update_;  // bitmask controlling alignment to road surface and orientation, update operations. See PositionMode enum.
        int    mode_set_;     // bitmask controlling alignment to road surface and orientation, set operations. See PositionMode enum.
        int    mode_init_;    // bitmask controlling alignment to road surface and orientation, initialization. See PositionMode enum.

        Position     *rel_pos_;
        PositionType  type_;
        DirectionMode direction_mode_;

        OrientationType orientation_type_;  // Applicable for relative positions
        int             snapToLaneTypes_;   // Bitmask of lane types that the position will snap to
        int             status_;            // Bitmask of various states, e.g. off_road, end_of_road

        // inertial reference
        double x_;
        double y_;
        double z_;
        double h_;
        double p_;
        double r_;
        double h_rate_;
        double p_rate_;
        double r_rate_;
        double h_acc_;
        double p_acc_;
        double r_acc_;
        double velX_;
        double velY_;
        double velZ_;
        double accX_;
        double accY_;
        double accZ_;
        double z_road_;
        double p_road_;
        double r_road_;
        double z_roadPrim_;              // the road vertical slope (dz/ds)
        double z_roadPrimPrim_;          // rate of change of the road slope, like the vertical curvature
        double roadSuperElevationPrim_;  // rate of change of the road superelevation/lateral inclination

        // keep track for fast incremental updates of the position
        int track_idx_;            // road index
        int lane_idx_;             // lane index
        int roadmark_idx_;         // laneroadmark index
        int roadmarktype_idx_;     // laneroadmark index
        int roadmarkline_idx_;     // laneroadmarkline index
        int lane_section_idx_;     // lane section
        int geometry_idx_;         // index of the segment within the track given by track_idx
        int elevation_idx_;        // index of the current elevation entry
        int super_elevation_idx_;  // index of the current super elevation entry
        int osi_point_idx_;        // index of the current closest OSI road point

        // RouteStrategy for a position, used for waypoints
        RouteStrategy routeStrategy_ = RouteStrategy::SHORTEST;

        // Store roads overlapping position, updated by XYZ2TrackPos()
        std::vector<int> overlapping_roads;  // road ids overlapping position evaluated by XYZ2TrackPos()

        int  orientationSetMask;  // use values from OrientationSetMask
        bool zSet;                // indicates whether z was explicitly set
    };

    // A route is a sequence of positions, at least one per road along the route
    class Route
    {
    public:
        Route() : invalid_route_(false), waypoint_idx_(-1), path_s_(0), length_(0)
        {
        }

        /**
        Adds a waypoint to the route. One waypoint per road. At most one junction between waypoints.
        @param position A regular position created with road, lane or world coordinates
        @return Non zero return value indicates error of some kind
        */
        int AddWaypoint(Position *position);
        int GetWayPointDirection(int index);

        void        setName(std::string name);
        std::string getName() const;
        double      GetLength() const
        {
            return length_;
        }
        void CheckValid();
        bool IsValid() const
        {
            return !invalid_route_;
        }

        // Current route position data
        // Actual object position might differ, e.g. laneId or even trackId in junctions
        double GetTrackS() const
        {
            return currentPos_.GetS();
        }
        double GetPathS() const
        {
            return path_s_;
        }
        int GetLaneId() const
        {
            return currentPos_.GetLaneId();
        }
        int GetTrackId() const
        {
            return currentPos_.GetTrackId();
        }
        Position *GetWaypoint(int index = -1);  // -1 means current
        Road     *GetRoadAtOtherEndOfConnectingRoad(Road *incoming_road) const;
        Position *GetCurrentPosition()
        {
            return &currentPos_;
        }

        /**
        Specify route position in terms of a track ID and track S value
        @return Non zero return value indicates error of some kind
        */
        Position::ReturnCode SetTrackS(int trackId, double s);

        /**
        Move current position forward, or backwards, ds meters along the route
        @param ds Distance to move, negative will move backwards
        @param actualDistance Distance considering lateral offset and curvature (true/default) or along centerline (false)
        @return Non zero return value indicates error of some kind, most likely End Of Route
        */
        Position::ReturnCode MovePathDS(double ds);

        /**
        Move current position to specified S-value along the route
        @param route_s Distance to move, negative will move backwards
        @return Non zero return value indicates error of some kind, most likely End Of Route
        */
        Position::ReturnCode SetPathS(double s);

        Position::ReturnCode CopySFractionOfLength(Position *pos);

        std::vector<Position> scenario_waypoints_;  // contains waypoints defined in .xosc file
        std::vector<Position> minimal_waypoints_;   // used only for the default controllers
        std::vector<Position> all_waypoints_;       // used for user-defined controllers
        std::string           name_;
        bool                  invalid_route_;
        double                path_s_;
        Position              currentPos_;
        double                length_;
        int                   waypoint_idx_;
    };

    // A Road Path is a linked list of road links (road connections or junctions)
    // between a starting position and a target position
    // The path can be calculated automatically
    class RoadPath
    {
    public:
        struct PathNode
        {
            RoadLink        *link       = 0;
            double           dist       = 0.0;
            Road            *fromRoad   = 0;
            int              fromLaneId = 0;
            ContactPointType contactPoint;
            PathNode        *previous  = 0;
            int              direction = 0;
        };

        std::vector<PathNode *> visited_;
        std::vector<PathNode *> unvisited_;
        const Position         *startPos_;
        const Position         *targetPos_;
        int                     direction_;  // direction of path from starting pos. 0==not set, 1==forward, 2==backward
        PathNode               *firstNode_;

        RoadPath(const Position *startPos, const Position *targetPos)
            : startPos_(startPos),
              targetPos_(targetPos),
              direction_(0),
              firstNode_(nullptr){};
        ~RoadPath();

        /**
        Calculate shortest path between starting position and target position,
        using Dijkstra's algorithm https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
        it also calculates the length of the path, or distance between the positions
        positive distance means that the shortest path was found in forward direction
        negative distance means that the shortest path goes in opposite direction from the heading of the starting position
        @param dist A reference parameter into which the calculated path distance is stored
        @param bothDirections Set to true in order to search also backwards from object
        @param maxDist If set the search along each path branch will terminate after reaching this distance
        @return 0 on success, -1 on failure e.g. path not found
        */
        int Calculate(double &dist, bool bothDirections = true, double maxDist = LARGE_NUMBER);

    private:
        bool CheckRoad(Road *checkRoad, RoadPath::PathNode *srcNode, Road *fromRoad, int fromLaneId);
    };

    class PolyLineBase
    {
    public:
        PolyLineBase() : length_(0), current_index_(0), current_s_(0.0)
        {
        }

        /**
         * Add vertex
         * @param v Vertex to add. Set s=std::nan("") to have s value automatically calculated.
         */
        void AddVertex(TrajVertex v);

        void reset()
        {
            length_ = 0.0;
        }
        int Evaluate(double s, TrajVertex &pos, double cornerRadius, int startAtIndex);
        int Evaluate(double s, TrajVertex &pos, double cornerRadius);
        int Evaluate(double s, TrajVertex &pos, int startAtIndex);
        int Evaluate(double s, TrajVertex &pos);
        int FindClosestPoint(double xin, double yin, TrajVertex &pos, int &index, int startAtIndex = 0);
        int FindPointAhead(double s_start, double distance, TrajVertex &pos, int &index, int startAtIndex = 0);

        /**
         * Get ghost state at a point in time
         * @param time Simulation time (subtracting headstart time, i.e. time=0 gives the initial state)
         * @param pos Returns state including position, heading, speed. See TrajVertex type.
         * @param index Returns the index of matching trajectory segment
         * @param startAtIndex Start search for segment (e.g. index returned by previous call)
         */
        int FindPointAtTime(double time, TrajVertex &pos, int &index, int startAtIndex = 0);

        /**
         * Get ghost state at a point in time
         * @param time Time offset from first timestamp
         * @param pos Returns state including position, heading, speed. See TrajVertex type.
         * @param index Returns the index of matching trajectory segment
         * @param startAtIndex Start search for segment (e.g. index returned by previous call)
         */
        int FindPointAtTimeRelative(double time, TrajVertex &pos, int &index, int startAtIndex = 0);

        int GetNumberOfVertices() const
        {
            return (int)vertex_.size();
        }
        TrajVertex *GetVertex(int index);
        TrajVertex *GetCurrentVertex();
        void        Reset(bool clear_vertices);
        int         Time2S(double time, double &s);

        std::vector<TrajVertex> vertex_;
        int                     current_index_;
        double                  current_s_;
        double                  length_;

    protected:
        int EvaluateSegmentByLocalS(int i, double local_s, double cornerRadius, TrajVertex &pos);
    };

    // Trajectory stuff
    class Shape
    {
    public:
        typedef enum
        {
            POLYLINE,
            CLOTHOID,
            CLOTHOID_SPLINE,
            NURBS,
            SHAPE_TYPE_UNDEFINED
        } ShapeType;

        typedef enum
        {
            TRAJ_PARAM_TYPE_S,
            TRAJ_PARAM_TYPE_TIME
        } TrajectoryParamType;

        Shape(ShapeType type) : type_(type)
        {
        }
        virtual ~Shape() = default;
        virtual int Evaluate(double p, TrajectoryParamType ptype, TrajVertex &pos)
        {
            return -1;
        };

        virtual void CalculatePolyLine()
        {
        }

        /**
                Find point on shape closest to provided point
                @param xin X coordinate of input position
                @param yin Y coordinate of input position
                @param pos Output parameter: Closest trajectory position
                @param index Output parameter: Index of closest underlying polyline segment, can be used as start index in next call
                @param startAtIndex: <= 0: Search global minimum along the whole path, > 0: Look for local minimum around this index.
                @return true if connection exist, else false
        */
        int FindClosestPoint(double xin, double yin, TrajVertex &pos, int &index, int startAtIndex = 0)
        {
            return -1;
        };
        virtual double GetLength()
        {
            return 0.0;
        }
        virtual double GetStartTime()
        {
            return 0.0;
        }
        virtual double GetDuration()
        {
            return 0.0;
        }
        ShapeType     type_;
        FollowingMode following_mode_;
        double        initial_speed_;
        PolyLineBase  pline_;  // approximation of shape, used for calculations and visualization
    };

    class PolyLineShape : public Shape
    {
    public:
        class Vertex
        {
        public:
            Vertex(const Position &pos, double time) : pos_(pos), time_(time)
            {
            }
            Position pos_;
            double   time_;
        };

        PolyLineShape() : Shape(ShapeType::POLYLINE)
        {
        }
        void   AddVertex(Position pos, double time);
        int    Evaluate(double p, TrajectoryParamType ptype, TrajVertex &pos);
        void   CalculatePolyLine() override;
        double GetLength()
        {
            return pline_.length_;
        }
        double GetStartTime();
        double GetDuration();

        std::vector<Vertex> vertex_;
    };

    class ClothoidShape : public Shape
    {
    public:
        ClothoidShape(roadmanager::Position pos, double curv, double curvDot, double len, double tStart, double tEnd);

        int    Evaluate(double p, TrajectoryParamType ptype, TrajVertex &pos);
        int    EvaluateInternal(double s, TrajVertex &pos);
        void   CalculatePolyLine() override;
        double GetLength()
        {
            return spiral_.GetLength();
        }
        double GetStartTime();
        double GetDuration();

        Position            pos_;
        roadmanager::Spiral spiral_;  // make use of the OpenDRIVE clothoid definition
        double              t_start_;
        double              t_end_;
    };

    class ClothoidSplineShape : public Shape
    {
    public:
        class Segment
        {
        public:
            Segment(Position *posStart, double curvStart, double curvEnd, double length, double h_offset, double time)
                : curvStart_(curvStart),
                  curvEnd_(curvEnd),
                  length_(length),
                  h_offset_(h_offset),
                  time_(time)
            {
                if (posStart != nullptr)
                {
                    posStart_ = new Position(*posStart);
                }
                else
                {
                    posStart_ = nullptr;
                }
            }

            Position *posStart_;
            Position  posEnd_;
            double    curvStart_;
            double    curvEnd_;
            double    length_;
            double    h_offset_;
            double    time_;
        };

        ClothoidSplineShape() : Shape(ShapeType::CLOTHOID_SPLINE)
        {
        }
        ~ClothoidSplineShape();

        void   AddSegment(Position *posStart, double curvStart, double curvEnd, double length, double h_offset, double time);
        int    Evaluate(double p, TrajectoryParamType ptype, TrajVertex &pos);
        int    EvaluateInternal(double s, int segment_idx, TrajVertex &pos);
        void   CalculatePolyLine();
        double GetLength()
        {
            return length_;
        }
        double GetStartTime();
        double GetDuration();
        double GetEndTime()
        {
            return time_end_;
        }
        void SetEndTime(double time)
        {
            time_end_ = time;
        }
        void Freeze(Position *ref_pos);
        int  GetNumberOfSegments()
        {
            return static_cast<int>(segments_.size());
        }

    private:
        std::vector<Segment>             segments_;
        std::vector<roadmanager::Spiral> spirals_;  // make use of the OpenDRIVE clothoid definition
        double                           length_   = 0.0;
        double                           time_end_ = 0.0;
    };

    /**
            This nurbs implementation is strongly inspired by the "Nurbs Curve Example" at:
            https://nccastaff.bournemouth.ac.uk/jmacey/OldWeb/RobTheBloke/www/opengl_programming.html
    */
    class NurbsShape : public Shape
    {
        class ControlPoint
        {
        public:
            Position pos_;
            double   time_;
            double   weight_;
            double   t_;

            ControlPoint(Position pos, double time, double weight) : pos_(pos), time_(time), weight_(weight)
            {
            }
        };

    public:
        NurbsShape(unsigned int order) : order_(order), Shape(ShapeType::NURBS), length_(0)
        {
        }

        void AddControlPoint(Position pos, double time, double weight);
        void AddKnots(std::vector<double> knots);
        int  Evaluate(double p, TrajectoryParamType ptype, TrajVertex &pos);
        int  EvaluateInternal(double s, TrajVertex &pos);

        unsigned int              order_;
        std::vector<ControlPoint> ctrlPoint_;
        std::vector<double>       knot_;
        std::vector<double>       d_;           // used for temporary storage of CoxDeBoor weigthed control points
        std::vector<double>       dPeakT_;      // used for storage of at what t value the corresponding ctrlPoint contribution peaks
        std::vector<double>       dPeakValue_;  // used for storage of at what t value the corresponding ctrlPoint contribution peaks

        void   CalculatePolyLine() override;
        double GetLength()
        {
            return length_;
        }
        double GetStartTime();
        double GetDuration();

    private:
        double CoxDeBoor(double x, int i, int p, const std::vector<double> &t);
        double length_;
    };

    class RMTrajectory
    {
    public:
        std::unique_ptr<Shape> shape_;

        RMTrajectory(std::unique_ptr<Shape> shape, std::string name, bool closed) : shape_(std::move(shape)), name_(name), closed_(closed)
        {
        }
        RMTrajectory() : closed_(false)
        {
        }

        void   Freeze(FollowingMode following_mode, double current_speed, Position *ref_pos = nullptr);
        double GetLength()
        {
            return shape_ ? shape_->GetLength() : 0.0;
        }
        double GetTimeAtS(double s);
        double GetStartTime();
        double GetDuration();

        std::string name_;
        bool        closed_;
    };

}  // namespace roadmanager

#endif  // OPENDRIVE_HH_
