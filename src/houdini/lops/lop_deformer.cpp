#include <UT/UT_DSOVersion.h>
#include <OP/OP_OperatorTable.h>
#include <OP/OP_Director.h>
#include <PRM/PRM_Include.h>
#include <LOP/LOP_Node.h>
#include <LOP/LOP_Error.h>

#include <HUSD/HUSD_AutoWriteLock.h>
#include <HUSD/HUSD_AutoReadLock.h>

// Pixar USD API Imports
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>

static PRM_Name sPrmnames[] = {
    PRM_Name("deformer_prim",     "Deformer Prim"),
};


class LOP_CustomExample : public LOP_Node
{
public:
    static OP_Node *myConstructor(OP_Network *net, const char *name, OP_Operator *op)
    {
        return new LOP_CustomExample(net, name, op);
    }

    static PRM_Template myTemplateList[];

protected:
    LOP_CustomExample(OP_Network *net, const char *name, OP_Operator *op)
        : LOP_Node(net, name, op) {}
    
    ~LOP_CustomExample() override {}

    // Main USD cooking function required for LOP nodes
    OP_ERROR cookMyLop(OP_Context &context) override;
};

// Define UI Parameters (e.g., Target Primitive Path)
PRM_Template LOP_CustomExample::myTemplateList[] = {
    PRM_Template(PRM_STRING, 1, &sPrmnames[0]),
    PRM_Template() // Sentinel marking the end of the array
};

OP_ERROR LOP_CustomExample::cookMyLop(OP_Context &context)
{
    // 1. Duplicate the stage from the first input or start a fresh one if missing
    //if (cookLOP(context) != UT_ERROR_NONE)
    //    return error();

    // 2. Evaluate parameter data safely at the given context evaluation time
    fpreal t = context.getTime();
    UT_String prim_path_ut;
    evalString(prim_path_ut, "name", 0, t);
    std::string primPathStr = prim_path_ut.toStdString();

    if (primPathStr.empty() || primPathStr[0] != '/') {
        addError(LOP_PRIM_NOT_FOUND, "Primitive path must begin with a forward slash '/'");
        return error();
    }

    // 3. Retrieve the underlying editable USD Stage layer 
    PxrUsdDataStageRefPtr stageRef = editableStage();
    if (!stageRef) {
        addError(LOP_ERR_STAGE_NOT_FOUND, "Failed to retrieve editable target stage.");
        return error();
    }
    pxr::UsdStageRefPtr stage = stageRef->stage();

    // 4. Inject USD modifications using standard Pixar USD APIs
    pxr::SdfPath path(primPathStr);
    pxr::UsdGeomXform xformPrim = pxr::UsdGeomXform::Define(stage, path);
    
    if (!xformPrim) {
        addError(LOP_ERR_CANT_CREATE_PRIM, "Failed to define Xform primitive on layer.");
        return error();
    }

    // 5. Explicitly log modified primitives to ensure downstream viewport updates
    UT_StringArray modifiedPaths;
    modifiedPaths.append(primPathStr.c_str());
    setLastModifiedPrims(modifiedPaths);

    return error();
}

// Register the custom LOP operator inside the operator table
void newLopOperator(OP_OperatorTable *table) {
    table->addOperator(new OP_Operator(
        "custom_cpp_example",                  // Internal unique operator type name
        "Custom C++ Example",                 // UI visible label
        LOP_CustomExample::myConstructor,     // Node constructor pointer
        LOP_CustomExample::myTemplateList,    // Parameter template array
        1,                                    // Minimum required inputs
        1                                     // Maximum possible inputs
    ));
}
