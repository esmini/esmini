#include <stdio.h>
#include <osg/Node>
#include <osg/StateSet>
#include "roadgeom.hpp"

void PrintNodeRecursive(const osg::Node* node)
{
    static int indent = 0;

    if (node == nullptr)
    {
        return;
    }
    else
    {
        printf("%*s%s\n", 2 * indent, "", node->getName().empty() ? "no-name" : node->getName().c_str());

        const osg::StateSet* ss = node->getStateSet();
        if (ss != nullptr)
        {
            const osg::Material* material = dynamic_cast<const osg::Material*>(ss->getAttribute(osg::StateAttribute::MATERIAL));
            if (material)
            {
                printf("%*smaterial: %s\n", 2 * (indent + 1), "", material->getName().empty() ? "no-name" : material->getName().c_str());
            }
        }

        const osg::Group* group = dynamic_cast<const osg::Group*>(node);
        if (group != nullptr)
        {
            indent++;
            for (unsigned int i = 0; i < group->getNumChildren(); i++)
            {
                PrintNodeRecursive(group->getChild(i));
            }
            indent--;
        }
    }
}

int main(int argc, char* argv[])
{
    std::string filename = "../../../../resources/xodr/crest-curve.xodr";  // default file

    if (argc > 1)
    {
        filename = argv[1];
    }

    SE_Env::Inst().GetOptions().SetOptionValue("path", DirNameOf(filename), true, true);

    if (!roadmanager::Position::LoadOpenDrive(filename.c_str()))
    {
        printf("Failed to load OpenDRIVE file %s\n", filename.c_str());
        return -1;
    }

    // Create road geometry from OpenDRIVE file
    roadgeom::RoadGeom road_geom = roadgeom::RoadGeom(roadmanager::Position::GetOpenDrive(), nullptr, osg::Vec3(0, 0, 0), true, true, argv[0], false);

    // Inspect slightly and print structure
    PrintNodeRecursive(road_geom.root_);

    // Save as OpenSceneGraph binary file
    std::string output = FileNameWithoutExtOf(filename) + ".osgb";
    road_geom.SaveToFile(output);

    printf("Generated model from %s saved as %s\n", FileNameOf(filename).c_str(), output.c_str());

    return 0;
}
