/* Copyright (C) 2023-2026 CascadiaVoxel LLC

    nanoPRC is free software: you can redistribute it and/or modify it under
    the terms of the GNU Affero General Public License as published by the
    Free Software Foundation, either version 3 of the License, or (at your
    option) any later version.

    nanoPRC is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public
    License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with nanoPRC. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef PRC_DATA_H
#define PRC_DATA_H

#include <stdint.h>
#include <stddef.h>
#include "prc_context.h"
#include "prc_vector_util.h"
#include "prc_internal_api.h"
#include "prc_pdf.h"
#include "prc_fwd.h"

#define prc_unsigned_int uint32_t
#define prc_bool uint8_t
#define true 1
#define false 0

#define READ_TAG 1
#define DONT_READ_TAG 0

#define prc_misc_attribute_MAX 20
#define prc_misc_attribute_TITLE 2

static const char *const prc_misc_attribute_NAMES[prc_misc_attribute_MAX] = {
  "", "", "Title", "Subject", "Author", "Keywords",
  "Comments", "Template", "Last Saved By", "Revision Number",
  "Total Editing Time", "Last Printed", "Create Time/Date",
   "Last saved Time/Date", "Number of Pages", "Number of Words",
   "Number of Characters", "Thumbnail", "Name of Creating Application",
   "Security"
};

/* Table 329 Schema tokens */
typedef enum {
    EPRCSchema_Data_Boolean = 0,
    EPRCSchema_Data_Double,
    EPRCSchema_Data_Character,
    EPRCSchema_Data_Unsigned_Integer,
    EPRCSchema_Data_Integer,
    EPRCSchema_Data_String,
    EPRCSchema_Parent_Type,
    EPRCSchema_Vector_2D,
    EPRCSchema_Vector_3D,
    EPRCSchema_Extent_1D,
    EPRCSchema_Extent_2D,
    EPRCSchema_Extent_3D,
    EPRCSchema_Ptr_Type,
    EPRCSchema_Ptr_Surface,
    EPRCSchema_Ptr_Curve,
    EPRCSchema_For,
    EPRCSchema_SimpleFor,
    EPRCSchema_If,
    EPRCSchema_Else,
    EPRCSchema_Block_Start,
    EPRCSchema_Block_Version,
    EPRCSchema_Block_End,
    EPRCSchema_Value_Declare,
    EPRCSchema_Value_Set,
    EPRCSchema_Value_DeclareAndSet,
    EPRCSchema_Value,
    EPRCSchema_Value_Constant,
    EPRCSchema_Value_For,
    EPRCSchema_Value_CurveIs3D,
    EPRCSchema_Operator_MULT,
    EPRCSchema_Operator_DIV,
    EPRCSchema_Operator_ADD,
    EPRCSchema_Operator_SUB,
    EPRCSchema_Operator_LT,
    EPRCSchema_Operator_LE,
    EPRCSchema_Operator_GT,
    EPRCSchema_Operator_GE,
    EPRCSchema_Operator_EQ,
    EPRCSchema_Operator_NEQ,
    EPRCSchema_Operator_IGNORE1,
    EPRCSchema_Operator_IGNORE2,
    EPRCSchema_PARENT_TYPE /* Used only for data reading */
} prc_schema_tokens_t;

/* Table 321 — Mathematical operator entity types */
typedef enum
{
    PRC_TYPE_MATH = 900,
    PRC_TYPE_MATH_FCT_1D,
    PRC_TYPE_MATH_FCT_1D_Polynom,
    PRC_TYPE_MATH_FCT_1D_Trigonometric,
    PRC_TYPE_MATH_FCT_1D_Fraction,
    PRC_TYPE_MATH_FCT_1D_ArctanCos,
    PRC_TYPE_MATH_FCT_1D_Combination,
    PRC_TYPE_MATH_FCT_3D,
    PRC_TYPE_MATH_FCT_3D_Linear,
    PRC_TYPE_MATH_FCT_3D_nonLinear
} prc_math_operator_entity_type;

/* Table 274 — EPRCIntersectionLimitType */
typedef enum
{
    PRC_KEPRCIntersectionLimitTypeHelp = 0,
    PRC_KEPRCIntersectionLimitTypeTerminator = 1,
    PRC_KEPRCIntersectionLimitTypeLimit = 2,
    PRC_KEPRCIntersectionLimitTypeBoundary = 3
} prc_eprc_intersection_limit_type;

/* Table 251 — EPRCKnotType */
typedef enum
{
    PRC_KEPRCKnotTypeUniformKnots = 0,
    PRC_KEPRCKnotTypeUnspecified = 1,
    PRC_KEPRCKnotTypeQuasiUniformKnots = 2,
    PRC_KEPRCKnotTypePiecewiseBezierKnot = 3
} prc_eprc_knot_type;

/* Table 252 — EPRCBsplineCurveForm */
typedef enum
{
    PRC_KEPRCBSplineCurveFormUnspecified = 0,
    PRC_KEPRCBSplineCurveFormPolylinel = 1,
    PRC_KEPRCBSplineCurveFormCircularArc = 2,
    PRC_KEPRCBSplineCurveFormEllipticArc = 3,
    PRC_KEPRCBSplineCurveFormParabolicArc = 4,
    PRC_KEPRCBSplineCurveFormHyperbolicArc = 5
} prc_eprc_bspline_curve_form;

/* Table 253 — EPRCExtendType */
typedef enum
{
    PRC_KEPRCExtendTypeNone = 0,
    PRC_KEPRCExtendTypeExt1 = 2,
    PRC_KEPRCExtendTypeExt2 = 4,
    PRC_KEPRCExtendTypeG1 = 6,
    PRC_KEPRCExtendTypeG1R = 8,
    PRC_KEPRCExtendTypeG1_G2 = 10,
    PRC_KEPRCExtendTypeCInfinity = 12
} prc_eprc_extend_type;

/* Table 72 */
typedef enum {
    PRC_MISC_ATTRIBUTE_TITLE = 2,
    PRC_MISC_ATTRIBUTE_SUBJECT,
    PRC_MISC_ATTRIBUTE_AUTHOR,
    PRC_MISC_ATTRIBUTE_KEYWORDS,
    PRC_MISC_ATTRIBUTE_COMMENTS,
    PRC_MISC_ATTRIBUTE_TEMPLATE,
    PRC_MISC_ATTRIBUTE_LAST_SAVED_BY,
    PRC_MISC_ATTRIBUTE_REVISION_NUM,
    PRC_MISC_ATTRIBUTE_TOTAL_EDIT_TIME,
    PRC_MISC_ATTRIBUTE_LAST_PRINTED,
    PRC_MISC_ATTRIBUTE_CREATED_TIME,
    PRC_MISC_ATTRIBUTE_LAST_SAVE_TIME,
    PRC_MISC_ATTRIBUTE_NUMBER_PAGES,
    PRC_MISC_ATTRIBUTE_NUMBER_WORDS,
    PRC_MISC_ATTRIBUTE_NUMBER_CHARACTERS,
    PRC_MISC_ATTRIBUTE_THUMBNAIL,
    PRC_MISC_ATTRIBUTE_NAME_CREATING_APPLICATION,
    PRC_MISC_ATTRIBUTE_SECURITY
} prc_misc_attribute_t;

/* Table 139 */
typedef enum {
    PRC_FACETESSDATA_NORMAL_Single = 0x40000000,
    PRC_FACETESSDATA_Polyface = 0x1,
    PRC_FACETESSDATA_Triangle = 0x2,
    PRC_FACETESSDATA_TriangleFan = 0x4,
    PRC_FACETESSDATA_TriangleStripe = 0x8,
    PRC_FACETESSDATA_PolyfaceOneNormal = 0x10,
    PRC_FACETESSDATA_TriangleOneNormal = 0x20,
    PRC_FACETESSDATA_TriangleFanOneNormal = 0x40,
    PRC_FACETESSDATA_TriangleStripeOneNormal = 0x80,
    PRC_FACETESSDATA_PolyfaceTextured = 0x100,
    PRC_FACETESSDATA_TriangleTextured = 0x200,
    PRC_FACETESSDATA_TriangleFanTextured = 0x400,
    PRC_FACETESSDATA_TriangleStripeTextured = 0x800,
    PRC_FACETESSDATA_PolyfaceOneNormalTextured = 0x1000,
    PRC_FACETESSDATA_TriangleOneNormalTextured = 0x2000,
    PRC_FACETESSDATA_TriangleFanOneNormalTextured = 0x4000,
    PRC_FACETESSDATA_TriangleStripeOneNormalTextured = 0x8000
} prc_face_attribute_t;

typedef enum {
    PRC_ATTRIBUTE_TYPE_INT = 1,
    PRC_ATTRIBUTE_TYPE_DOUBLE,
    PRC_ATTRIBUTE_TYPE_TIME32,
    PRC_ATTRIBUTE_TYPE_CHAR_UTF8,
    PRC_ATTRIBUTE_TYPE_TIME64
} prc_attribute_key_t;

typedef enum {
    PRC_TYPE_ROOT = 0,
    PRC_TYPE_ROOTBase,
    PRC_TYPE_ROOTBaseWithGraphics,
    PRC_TYPE_ROOTBaseNoReference
} prc_entity_t;

/* Table 35 */
typedef enum {
    PRC_TYPE_ASM = 300,
    PRC_TYPE_ASM_ModelFile,
    PRC_TYPE_ASM_FileStructure,
    PRC_TYPE_ASM_FileStructureGlobals,
    PRC_TYPE_ASM_FileStructureTree,
    PRC_TYPE_ASM_FileStructureTessellation,
    PRC_TYPE_ASM_FileStructureGeometry,
    PRC_TYPE_ASM_FileStructureExtraGeometry,
    PRC_TYPE_ASM_ProductOccurrence = PRC_TYPE_ASM + 10,
    PRC_TYPE_ASM_PartDefinition,
    PRC_TYPE_ASM_Filter = PRC_TYPE_ASM + 20,
} prc_assembly_t;

/* Table 70 Miscellaneous data entity types */
typedef enum {
    PRC_TYPE_MISC = PRC_TYPE_ROOT + 200,
    PRC_TYPE_MISC_Attribute,
    PRC_TYPE_MISC_CartesianTransformation,
    PRC_TYPE_MISC_EntityReference,
    PRC_TYPE_MISC_MarkupLinkedItem,
    PRC_TYPE_MISC_ReferenceOnPRCBase,
    PRC_TYPE_MISC_ReferenceOnTopology,
    PRC_TYPE_MISC_GeneralTransformation
} prc_miscellaneous_t;

/* Table 136 Tessellation entity types */
typedef enum {
    PRC_TYPE_TESS = PRC_TYPE_ROOT + 170,
    PRC_TYPE_TESS_Base,
    PRC_TYPE_TESS_3D,
    PRC_TYPE_TESS_3D_Compressed,
    PRC_TYPE_TESS_Face,
    PRC_TYPE_TESS_3D_Wire,
    PRC_TYPE_TESS_MarkUp
} prc_tesslation_t;

/* Table 89 Graphics entity types */
typedef enum {
    PRC_TYPE_GRAPH = PRC_TYPE_ROOT + 700,
    PRC_TYPE_GRAPH_Style,
    PRC_TYPE_GRAPH_Material,
    PRC_TYPE_GRAPH_Picture,
    PRC_TYPE_GRAPH_TextureApplication = PRC_TYPE_GRAPH + 11,
    PRC_TYPE_GRAPH_TextureDefinition,
    PRC_TYPE_GRAPH_TextureTransformation,
    PRC_TYPE_GRAPH_LinePattern = PRC_TYPE_GRAPH + 21,
    PRC_TYPE_GRAPH_FillPattern,
    PRC_TYPE_GRAPH_DottingPattern,
    PRC_TYPE_GRAPH_HatchingPattern,
    PRC_TYPE_GRAPH_SolidPattern,
    PRC_TYPE_GRAPH_VpicturePattern,
    PRC_TYPE_GRAPH_AmbientLight = PRC_TYPE_GRAPH + 31,
    PRC_TYPE_GRAPH_PointLight,
    PRC_TYPE_GRAPH_DirectionalLight,
    PRC_TYPE_GRAPH_SpotLight,
    PRC_TYPE_GRAPH_SceneDisplayParameters = PRC_TYPE_GRAPH + 41,
    PRC_TYPE_GRAPH_Camera
} prc_graphics_t;

/* Table 94 Picture data format */
typedef enum {
    KEPRCPicture_PNG = 0,
    KEPRCPicture_JPG,
    KEPRCPicture_BITMAP_RGB_BYTE,
    KEPRCPicture_BITMAP_RGBA_BYTE,
    KEPRCPicture_BITMAP_GREY_BYTE,
    KEPRCPicture_BITMAP_GREYA_BYTE
} prc_picture_data_format_t;

/* Table 129 Markup types */
typedef enum {
    KEPRCMarkupType_Unknown = 0,
    KEPRCMarkupType_Text,
    KEPRCMarkupType_Dimension,
    KEPRCMarkupType_Arrow,
    KEPRCMarkupType_Balloon,
    KEPRCMarkupType_CircleCenter,
    KEPRCMarkupType_Coordinate,
    KEPRCMarkupType_Datum,
    KEPRCMarkupType_Fastener,
    KEPRCMarkupType_Gdt,
    KEPRCMarkupType_Locator,
    KEPRCMarkupType_MeasurementPoint,
    KEPRCMarkupType_Roughness,
    KEPRCMarkupType_Welding,
    KEPRCMarkupType_Table,
    KEPRCMarkupType_Other
} prc_markup_types_t;

/* Table 130 Markup subtypes */
typedef enum {
    KEPRCMarkupSubType_Datum_Ident = 1,
    KEPRCMarkupSubType_Datum_Target,
    KEPRCMarkupSubType_Dimension_Distance,
    KEPRCMarkupSubType_Dimension_Distance_Offset,
    KEPRCMarkupSubType_Dimension_Distance_Cumulate,
    KEPRCMarkupSubType_Dimension_Chamfer,
    KEPRCMarkupSubType_Dimension_Slope,
    KEPRCMarkupSubType_Dimension_Ordinate,
    KEPRCMarkupSubType_Dimension_Radius,
    KEPRCMarkupSubType_Dimension_Radius_Tangent,
    KEPRCMarkupSubType_Dimension_Radius_Cylinder,
    KEPRCMarkupSubType_Dimension_Radius_Edge,
    KEPRCMarkupSubType_Dimension_Diameter,
    KEPRCMarkupSubType_Dimension_Diameter_Tangent,
    KEPRCMarkupSubType_Dimension_Diameter_Cylinder,
    KEPRCMarkupSubType_Dimension_Diameter_Edge,
    KEPRCMarkupSubType_Dimension_Diameter_Cone,
    KEPRCMarkupSubType_Dimension_Length,
    KEPRCMarkupSubType_Dimension_Length_Curvilinear,
    KEPRCMarkupSubType_Dimension_Length_Circular,
    KEPRCMarkupSubType_Dimension_Angle,
    KEPRCMarkupSubType_Gdt_Fcf,
    KEPRCMarkupSubType_Welding_Line,
    KEPRCMarkupSubType_Welding_Spot,
    KEPRCMarkupSubType_Other_Symbol_User,
    KEPRCMarkupSubType_Other_Symbol_Utility,
    KEPRCMarkupSubType_Other_Symbol_Custom,
    KEPRCMarkupSubType_Other_GeometricReference
} prc_markup_subtypes_t;

/* Table 115 � Representation items entity types */
typedef enum {
    PRC_TYPE_RI = PRC_TYPE_ROOT + 230,
    PRC_TYPE_RI_RepresentationalItem,
    PRC_TYPE_RI_BrepModel,
    PRC_TYPE_RI_Curve,
    PRC_TYPE_RI_Direction,
    PRC_TYPE_RI_Plane,
    PRC_TYPE_RI_PointSet,
    PRC_TYPE_RI_PolyBrepModel,
    PRC_TYPE_RI_PolyWire,
    PRC_TYPE_RI_Set,
    PRC_TYPE_RI_CoordinateSystem
} prc_representation_item_t;

/* Table 126 -- Markup entity types */
typedef enum {
    PRC_TYPE_MKP = PRC_TYPE_ROOT + 500,
    PRC_TYPE_MKP_View,
    PRC_TYPE_MKP_Markup,
    PRC_TYPE_MKP_Leader,
    PRC_TYPE_MKP_AnnotationItem,
    PRC_TYPE_MKP_AnnotationSet,
    PRC_TYPE_MKP_AnnotationReference
} prc_markup_type_entity_t;

/* Table 177 -- Topological entity types */
typedef enum {
    PRC_TYPE_TOPO = PRC_TYPE_ROOT + 140,
    PRC_TYPE_TOPO_Context,
    PRC_TYPE_TOPO_Item,
    PRC_TYPE_TOPO_MultipleVertex,
    PRC_TYPE_TOPO_UniqueVertex,
    PRC_TYPE_TOPO_WireEdge,
    PRC_TYPE_TOPO_Edge,
    PRC_TYPE_TOPO_CoEdge,
    PRC_TYPE_TOPO_Loop,
    PRC_TYPE_TOPO_Face,
    PRC_TYPE_TOPO_Shell,
    PRC_TYPE_TOPO_Connex,
    PRC_TYPE_TOPO_Body,
    PRC_TYPE_TOPO_SingleWireBody,
    PRC_TYPE_TOPO_BrepData,
    PRC_TYPE_TOPO_SingleWireBodyCompress,
    PRC_TYPE_TOPO_BrepDataCompress,
    PRC_TYPE_TOPO_WireBody
} prc_topological_entity_t;

/* These texture items are poorly defined in original spec */

/* Table 97 */
typedef enum {
    PRC_texture_mapping_choose = 1,
    PRC_texture_mapping_3D_tess,
    PRC_texture_mapping_retrieve_UV,
    PRC_texture_mapping_defined
} PRC_texture_mapping_t;

/* Table 98 */
typedef enum {
    PRC_texture_mapping_operator_unknown = 1,
    PRC_texture_mapping_operator_planar,
    PRC_texture_mapping_operator_cylindrical,
    PRC_texture_mapping_operator_spherical,
    PRC_texture_mapping_operator_cubic
} PRC_texture_mapping_operator_t;

/* Table 99 */
typedef enum {
    PRC_texture_mapping_attributes_red = 0x1,
    PRC_texture_mapping_attributes_green = 0x2,
    PRC_texture_mapping_attributes_blue = 0x4,
    PRC_texture_mapping_operator_rgb = 0x7,
    PRC_texture_mapping_attributes_alpha = 0x8,
    PRC_texture_mapping_attributes_rgba = 0xf
} PRC_texture_mapping_attributes_t;

/* Table 100 */
typedef enum {
    PRC_texture_function_unknown = 0x1,
    PRC_texture_function_modulate = 0x2,
    PRC_texture_function_replace = 0x3,
    PRC_texture_function_blend = 0x4,
    PRC_texture_function_decal = 0x5,
} PRC_texture_function_t;

/* Table 101 */
typedef enum {
    PRC_texture_application_app = 0x0,
    PRC_texture_application_lighting = 0x1,
    PRC_texture_application_alpha = 0x2,
    PRC_texture_application_combine = 0x4,
} PRC_texture_application_mode_t;

/* Table 102 */
typedef enum {
    PRC_texture_wrapping_application_choose = 1,
    PRC_texture_wrapping_repeat = 2,
    PRC_texture_wrapping_clamp_to_border = 3,
    PRC_texture_wrapping_clamp = 4,
    PRC_texture_wrapping_clamp_to_edge = 5,
    PRC_texture_wrapping_mirrored_repeat = 6
} PRC_texture_wrapping_mode_t;

/* Table 141 */
typedef enum {
    PRC_FACETESSDATA_WIRE_IsNotDrawn = 0x4000,
    PRC_FACETESSDATA_WIRE_IsClosing = 0x8000
} PRC_face_tessellations_flags_t;

/* Table 146 */
typedef enum {
    PRC_3DWIRETESSDATA_IsClosing = 0x10000000,
    PRC_3DWIRETESSDATA_IsContinuous = 0x20000000
} PRC_3d_wire_tess_flags_t;

/* Table 148 */
typedef enum {
    PRC_MARKUP_IsMatrix = 0x08000000, /* was 0x10000000 in original spec */
    PRC_MARKUP_IsExtraData = 0x04000000,
    PRC_MARKUP_IntegerMask = 0x000FFFFF,
    PRC_MARKUP_ExtraDataType = 0x03E00000
} PRC_test_markup_flags_t;

/* Table 75 */
typedef enum {
    PRC_transformation_identity = 0,
    PRC_transformation_translate,
    PRC_transformation_rotation,
    PRC_transformation_mirror = 0x04,
    PRC_transformation_scale = 0x08,
    PRC_transformation_nonuniform_scale = 0x10
} PRC_cartesian_transformation_name_t;

/* Table 33 */
typedef enum {
    PRC_GRAPHICS_Show = 0x0001,
    PRC_GRAPHICS_ChildHeritShow = 0x0002,
    PRC_GRAPHICS_FatherHeritShow = 0x0004,
    PRC_GRAPHICS_ChildHeritColor = 0x0008,
    PRC_GRAPHICS_ParentHeritColor = 0x0010,
    PRC_GRAPHICS_ChildHeritLayer = 0x0020,
    PRC_GRAPHICS_ParentHeritLayer = 0x0040,
    PRC_GRAPHICS_ChildHeritTransparency = 0x0080,
    PRC_GRAPHICS_ParentHeritTransparency = 0x0100,
    PRC_GRAPHICS_ChildHeritLinePattern = 0x0200,
    PRC_GRAPHICS_ParentHeritLinePattern = 0x0400,
    PRC_GRAPHICS_ChildHeritLineWidth = 0x0800,
    PRC_GRAPHICS_ParentHeritLineWidth = 0x1000,
    PRC_GRAPHICS_Removed = 0x2000,
} PRC_behavior_bit_field_t;

/* Table 149 */
typedef enum {
    PRC_MARKUP_IsHidden = 0x01,
    PRC_MARKUP_HasFrame = 0x02,
    PRC_MARKUP_IsNotModifiable = 0x04,
    PRC_MARKUP_IsZoomable = 0x08,
    PRC_MARKUP_IsOnTop = 0x10,
    PRC_MARKUP_IsFlipable = 0x20
} PRC_markup_tessellation_behavior_t;

/* Markup support */
typedef enum {
    PRC_NO_MODE = -1,
    PRC_FACE_VIEW_MODE = 0,
    PRC_FRAME_DRAW_MODE = 1,
    PRC_FIXED_SIZE_MODE = 2
} prc_markup_block_mode_t;

/* Table 179 */
typedef enum {
    PRC_CONTEXT_OuterLoopsFirst = 0x0001,
    PRC_CONTEXT_NoClamp = 0x0002,
    PRC_CONTEXT_NoSplit = 0x0004
} PRC_context_behavior_bit_values_t;

/* Table 245 */
typedef enum {
    PRC_TYPE_CRV = PRC_TYPE_ROOT + 10,
    PRC_TYPE_CRV_Base = PRC_TYPE_CRV + 1,
    PRC_TYPE_CRV_Blend02Boundary = PRC_TYPE_CRV + 2,
    PRC_TYPE_CRV_NURBS = PRC_TYPE_CRV + 3,
    PRC_TYPE_CRV_Circle = PRC_TYPE_CRV + 4,
    PRC_TYPE_CRV_Composite = PRC_TYPE_CRV + 5,
    PRC_TYPE_CRV_OnSurf = PRC_TYPE_CRV + 6,
    PRC_TYPE_CRV_Ellipse = PRC_TYPE_CRV + 7,
    PRC_TYPE_CRV_Equation = PRC_TYPE_CRV + 8,
    PRC_TYPE_CRV_Helix01 = PRC_TYPE_CRV + 9,
    PRC_TYPE_CRV_Hyperbola = PRC_TYPE_CRV + 10,
    PRC_TYPE_CRV_Intersection = PRC_TYPE_CRV + 11,
    PRC_TYPE_CRV_Line = PRC_TYPE_CRV + 12,
    PRC_TYPE_CRV_Offset = PRC_TYPE_CRV + 13,
    PRC_TYPE_CRV_Parabola = PRC_TYPE_CRV + 14,
    PRC_TYPE_CRV_PolyLine = PRC_TYPE_CRV + 15,
    PRC_TYPE_CRV_Transform = PRC_TYPE_CRV + 16
} PRC_curve_entity_types_t;

/* Table 284 */
typedef enum {
    PRC_TYPE_SURF = PRC_TYPE_ROOT + 75,
    PRC_TYPE_SURF_Base = PRC_TYPE_SURF + 1,
    PRC_TYPE_SURF_Blend01 = PRC_TYPE_SURF + 2,
    PRC_TYPE_SURF_Blend02 = PRC_TYPE_SURF + 3,
    PRC_TYPE_SURF_Blend03 = PRC_TYPE_SURF + 4,
    PRC_TYPE_SURF_NURBS = PRC_TYPE_SURF + 5,
    PRC_TYPE_SURF_Cone = PRC_TYPE_SURF + 6,
    PRC_TYPE_SURF_Cylinder = PRC_TYPE_SURF + 7,
    PRC_TYPE_SURF_Cylindrical = PRC_TYPE_SURF + 8,
    PRC_TYPE_SURF_Offset = PRC_TYPE_SURF + 9,
    PRC_TYPE_SURF_Pipe = PRC_TYPE_SURF + 10,
    PRC_TYPE_SURF_Plane = PRC_TYPE_SURF + 11,
    PRC_TYPE_SURF_Ruled = PRC_TYPE_SURF + 12,
    PRC_TYPE_SURF_Sphere = PRC_TYPE_SURF + 13,
    PRC_TYPE_SURF_Revolution = PRC_TYPE_SURF + 14,
    PRC_TYPE_SURF_Extrusion = PRC_TYPE_SURF + 15,
    PRC_TYPE_SURF_FromCurves = PRC_TYPE_SURF + 16,
    PRC_TYPE_SURF_Torus = PRC_TYPE_SURF + 17,
    PRC_TYPE_SURF_Transform = PRC_TYPE_SURF + 18,
    PRC_TYPE_SURF_Blend04 = PRC_TYPE_SURF + 19
} PRC_surface_entity_types_t;

/* Table 192 */
typedef enum {
    PRC_BODY_BBOX_Evaluation = 1,
    PRC_BODY_BBOX_Precise = 2,
    PRC_BODY_BBOX_CADData = 4
} PRC_Content_Body_BBOX_types_t;

/* Table 203 — Enumeration of Compressed Entity Types */
typedef enum {
    PRC_HCG_NewLoop = 0,
    PRC_HCG_EndLoop,
    PRC_HCG_IsoPlane,
    PRC_HCG_IsoCylinder,
    PRC_HCG_IsoTorus,
    PRC_HCG_IsoSphere,
    PRC_HCG_IsoCone,
    PRC_HCG_IsoNURBS,
    PRC_HCG_AnaPlane,
    PRC_HCG_AnaCylinder,
    PRC_HCG_AnaTorus,
    PRC_HCG_AnaSphere,
    PRC_HCG_AnaCone,
    PRC_HCG_AnaNURBS,
    PRC_HCG_AnaGenericFace
} PRC_Compressed_3D_t;

/* Table 203 — Enumeration of Compressed Entity Types */
typedef enum {
    PRC_HCG_Line = 0,
    PRC_HCG_Circle = 1,
    PRC_HCG_BsplineHermiteCurve = 2,
    PRC_HCG_Ellipse = 12,
    PRC_HCG_CompositeCurve = 13
} PRC_Compressed_curve_t;

/* Predefines for abstract types */
typedef struct prc_surf_blend01_s prc_surf_blend01;
typedef struct prc_surf_blend02_s prc_surf_blend02;
typedef struct prc_surf_blend03_s prc_surf_blend03;
typedef struct prc_surf_nurbs_s prc_surf_nurbs;
typedef struct prc_surf_cone_s prc_surf_cone;
typedef struct prc_surf_cylinder_s prc_surf_cylinder;
typedef struct prc_surf_cylindrical_s prc_surf_cylindrical;
typedef struct prc_surf_offset_s prc_surf_offset;
typedef struct pr_surf_pipe_s prc_surf_pipe;
typedef struct prc_surf_ruled_s prc_surf_ruled;
typedef struct prc_surf_sphere_s prc_surf_sphere;
typedef struct prc_surf_revolution_s prc_surf_revolution;
typedef struct prc_surf_extrusion_s prc_surf_extrusion;
typedef struct prc_surf_fromcurves_s prc_surf_fromcurves;
typedef struct prc_surf_torus_s prc_surf_torus;
typedef struct prc_surf_transform_s prc_surf_transform;
typedef struct prc_combination_functions_s prc_combination_functions;
typedef struct prc_math_fct_1d_s prc_math_fct_1d;
typedef struct prc_math_fct_3d_s prc_math_fct_3d;
typedef struct prc_math_fct_1d_polynom_s prc_math_fct_1d_polynom;
typedef struct prc_math_fct_1d_trigonometric_s prc_math_fct_1d_trigonometric;
typedef struct prc_math_fct_1d_fraction_s prc_math_fct_1d_fraction;
typedef struct prc_math_fct_1d_arctancos_s prc_math_fct_1d_arctancos;
typedef struct prc_math_fct_1d_combination_s prc_math_fct_1d_combination;
typedef struct prc_math_fct_3d_linear_s prc_math_fct_3d_linear;
typedef struct prc_math_fct_3d_nonlinear_s prc_math_fct_3d_nonlinear;
typedef struct prc_type0_helix_data_s prc_type0_helix_data;
typedef struct prc_type1_helix_data_s prc_type1_helix_data;
typedef struct prc_composite_subcurve_s prc_composite_subcurve;
typedef struct prc_crv_blend02_boundary_s prc_crv_blend02_boundary;
typedef struct prc_crv_nurbs_s prc_crv_nurbs;
typedef struct prc_control_points_nurbs_crv_s prc_control_points_nurbs_crv;
typedef struct prc_control_points_nurbs_surf_s prc_control_points_nurbs_surf;
typedef struct prc_crv_circle_s prc_crv_circle;
typedef struct prc_crv_composite_s prc_crv_composite;
typedef struct prc_crv_onsurf_s prc_crv_onsurf;
typedef struct prc_crv_ellipse_s prc_crv_ellipse;
typedef struct prc_crv_equation_s prc_crv_equation;
typedef struct prc_crv_helix01_s prc_crv_helix01;
typedef struct prc_crv_hyperbola_s prc_crv_hyperbola;
typedef struct prc_crv_intersection_s prc_crv_intersection;
typedef struct prc_crv_crossing_points_intersection_s prc_crv_crossing_points_intersection;
typedef struct prc_crv_line_s prc_crv_line;
typedef struct prc_crv_offset_s prc_crv_offset;
typedef struct prc_crv_parabola_s prc_crv_parabola;
typedef struct prc_polyline_point_s prc_polyline_point;
typedef struct prc_crv_polyline_s prc_crv_polyline;
typedef struct prc_crv_transform_s prc_crv_transform;
typedef struct prc_content_curve_s prc_content_curve;
typedef struct prc_content_wire_edge_s prc_content_wire_edge;
typedef struct prc_ref_or_compressed_curve_s prc_ref_or_compressed_curve;
typedef struct prc_content_compressed_face_s prc_content_compressed_face;
typedef struct prc_hcg_iso_plane_s prc_hcg_iso_plane;
typedef struct prc_hcg_iso_cylinder_s prc_hcg_iso_cylinder;
typedef struct prc_hcg_iso_torus_s prc_hcg_iso_torus;
typedef struct prc_hcg_iso_sphere_s prc_hcg_iso_sphere;
typedef struct prc_hcg_iso_cone_s prc_hcg_iso_cone;
typedef struct prc_hcg_iso_nurbs_s prc_hcg_iso_nurbs;
typedef struct prc_hcg_ana_plane_s prc_hcg_ana_plane;
typedef struct prc_hcg_ana_cylinder_s prc_hcg_ana_cylinder;
typedef struct prc_hcg_ana_torus_s prc_hcg_ana_torus;
typedef struct prc_hcg_ana_sphere_s prc_hcg_ana_sphere;
typedef struct prc_hcg_ana_cone_s prc_hcg_ana_cone;
typedef struct prc_hcg_ana_generic_face_s prc_hcg_ana_generic_face;
typedef struct prc_hcg_ana_nurbs_s prc_hcg_ana_nurbs;
typedef struct prc_tess_3d_wire_element_s prc_tess_3d_wire_element;
typedef struct prc_tess_3d_wire_s prc_tess_3d_wire;
typedef struct prc_tess_markup_s prc_tess_markup;
typedef struct prc_tess_3d_compressed_s prc_tess_3d_compressed;
typedef struct prc_graph_dotting_pattern_s prc_graph_dotting_pattern;
typedef struct prc_graph_hatching_pattern_s prc_graph_hatching_pattern;
typedef struct prc_graph_solid_pattern_s prc_graph_solid_pattern;
typedef struct prc_graph_picture_s prc_graph_picture;
typedef struct prc_tess_3d_s prc_tess_3d;
typedef struct prc_graph_vpicture_pattern_s prc_graph_vpicture_pattern;
typedef struct prc_ri_coordinate_system_s prc_ri_coordinate_system;
typedef struct prc_ri_brep_model_s prc_ri_brep_model;
typedef struct prc_ri_curve_s prc_ri_curve;
typedef struct prc_ri_direction_s prc_ri_direction;
typedef struct prc_ri_plane_s prc_ri_plane;
typedef struct prc_ri_point_set_s prc_ri_point_set;
typedef struct prc_ri_poly_brep_model_s prc_ri_poly_brep_model;
typedef struct prc_ri_poly_wire_s prc_ri_poly_wire;
typedef struct prc_ri_set_s prc_ri_set;
typedef struct prc_user_data_s prc_user_data;
typedef struct prc_bbox_s prc_bbox;
typedef struct prc_domain_s prc_domain;
typedef struct prc_uv_parameterization_s prc_uv_parameterization;
typedef struct prc_string_s prc_string;
typedef struct prc_unique_id_s prc_unique_id;
typedef struct prc_file_struct_desc_s prc_file_struct_desc;
typedef struct prc_uncomp_block_s prc_uncomp_block;
typedef struct prc_uncomp_file_s prc_uncomp_file;
typedef struct prc_file_structure_header_s prc_file_structure_header;
typedef struct prc_entity_schema_s prc_entity_schema;
typedef struct prc_schema_s prc_schema;
typedef struct prc_header_s prc_header;
typedef struct prc_attribute_entry_s prc_attribute_entry;
typedef struct prc_misc_attribute_s prc_misc_attribute;
typedef struct prc_attribute_data_s prc_attribute_data;
typedef struct prc_name_s prc_name;
typedef struct prc_content_prc_base_s prc_content_prc_base;
typedef struct prc_content_prc_ref_base_s prc_content_prc_ref_base;
typedef struct prc_font_key_s prc_font_key;
typedef struct prc_font_keys_same_font_s prc_font_keys_same_font;
typedef struct prc_markup_serialization_helper_s prc_markup_serialization_helper;
typedef struct prc_rgb_color_s prc_rgb_color;
typedef struct prc_graph_line_pattern_s prc_graph_line_pattern;
typedef struct prc_cart_transformation_s prc_cart_transformation;
typedef struct prc_misc_general_trans_s prc_misc_general_trans;
typedef struct prc_trans_3d_s prc_trans_3d;
typedef struct prc_cartesian_trans_2d_s prc_cartesian_trans_2d;
typedef struct prc_graph_texture_transformation_s prc_graph_texture_transformation;
typedef struct prc_graph_texture_definition_s prc_graph_texture_definition;
typedef struct prc_graph_material_s prc_graph_material;
typedef struct prc_graph_style_s prc_graph_style;
typedef struct prc_content_base_tess_data_s prc_content_base_tess_data;
typedef struct prc_hatch_s prc_hatch;
typedef struct prc_representation_item_content_s prc_representation_item_content;
typedef struct prc_file_struct_internal_global_data_s prc_file_struct_internal_global_data;
typedef struct prc_graphics_content_s prc_graphics_content;
typedef struct prc_content_entity_graphics_reference_s prc_content_entity_graphics_reference;
typedef struct prc_bounding_box_s prc_bounding_box;
typedef struct prc_additional_target_data_s prc_additional_target_data;
typedef struct prc_misc_reference_on_topology_s prc_misc_reference_on_topology;
typedef struct prc_misc_reference_on_prcbase_s prc_misc_reference_on_prcbase;
typedef struct prc_reference_data_s prc_reference_data;
typedef struct prc_content_extended_entity_ref_s prc_content_extended_entity_ref;
typedef struct prc_misc_markup_linked_item_s prc_misc_markup_linked_item;
typedef struct prc_mkp_leader_s prc_mkp_leader;
typedef struct prc_mkp_markup_s prc_mkp_markup;
typedef struct prc_annotation_entries_s prc_annotation_entries;
typedef struct prc_markup_data_s prc_markup_data;
typedef struct prc_content_entity_filter_item_s prc_content_entity_filter_item;
typedef struct prc_asm_filter_s prc_asm_filter;
typedef struct prc_content_surface_s prc_content_surface;
typedef struct prc_surf_plane_s prc_surf_plane;
typedef struct prc_graph_light_object_s prc_graph_light_object;
typedef struct prc_graph_camera_s prc_graph_camera;
typedef struct prc_scene_display_parameters_s prc_scene_display_parameters;
typedef struct prc_mkp_view_s prc_mkp_view;
typedef struct prc_asm_parts_definition_s prc_asm_parts_definition;
typedef struct prc_file_identifier_s prc_file_identifier;
typedef struct prc_references_of_product_occurrence_s prc_references_of_product_occurrence;
typedef struct prc_product_information_s prc_product_information;
typedef struct prc_content_entity_reference_s prc_content_entity_reference;
typedef struct prc_asm_product_occurrence_s prc_asm_product_occurrence;
typedef struct prc_color_data_remainder_s prc_color_data_remainder;
typedef struct prc_color_data_s prc_color_data;
typedef struct prc_vertex_colors_s prc_vertex_colors;
typedef struct prc_tess_face_s prc_tess_face;
typedef struct prc_binary_texture_data_s prc_binary_texture_data;
typedef struct prc_compressed_texture_parameter_s prc_compressed_texture_parameter;
typedef struct prc_asm_file_structure_tessellation_s prc_asm_file_structure_tessellation;
typedef struct prc_asm_file_structure_tree_s prc_asm_file_structure_tree;
typedef struct prc_asm_file_structure_globals_s prc_asm_file_structure_globals;
typedef struct prc_topo_context_s prc_topo_context;
typedef struct prc_file_structure_exact_geometry_s prc_file_structure_exact_geometry;
typedef struct prc_body_information_s prc_body_information;
typedef struct prc_element_information_s prc_element_information;
typedef struct prc_element_graphics_behavior_s prc_element_graphics_behavior;
typedef struct prc_graphics_information_s prc_graphics_information;
typedef struct prc_context_graphics_s prc_context_graphics;
typedef struct prc_geometry_summary_s prc_geometry_summary;
typedef struct prc_extra_geometry_s prc_extra_geometry;
typedef struct prc_asm_file_structure_extra_geometry_s prc_asm_file_structure_extra_geometry;
typedef struct prc_asm_file_structure_geometry_s prc_asm_file_structure_geometry;
typedef struct prc_filestructure_s prc_filestructure;
typedef struct prc_data_s prc_data;
typedef struct prc_triangle_s prc_triangle;
typedef struct prc_topo_wire_edge_s prc_topo_wire_edge;
typedef struct prc_topo_edge_s prc_topo_edge;
typedef struct prc_topo_single_wire_body_s prc_topo_single_wire_body;
typedef struct prc_topo_brep_data_s prc_topo_brep_data;
typedef struct prc_topo_single_wire_compress_s prc_topo_single_wire_compress;
typedef struct prc_topo_brep_data_compress_s prc_topo_brep_data_compress;
typedef struct prc_content_body_s prc_content_body;
typedef struct prc_base_topology_s prc_base_topology;
typedef struct prc_ptr_topology_s prc_ptr_topology;
typedef struct prc_hcg_bspline_hermite_curve_s prc_hcg_bspline_hermite_curve;
typedef struct prc_hcg_composite_curve_s prc_hcg_composite_curve;
typedef struct prc_compressed_vertex_s prc_compressed_vertex;
typedef struct prc_compressed_point_s prc_compressed_point;
typedef struct prc_compressed_shell_s prc_compressed_shell;
typedef struct prc_compressed_face_s prc_compressed_face;
typedef struct prc_multi_compressed_connex_s prc_multi_compressed_connex;
typedef struct prc_compressed_connex_s prc_compressed_connex;
typedef struct prc_type_asm_file_struct_internal_data_s prc_type_asm_file_struct_internal_data;
typedef struct prc_attribute_key_value_s prc_attribute_key_value;
typedef struct prc_reference_unique_identifiers_s prc_reference_unique_identifiers;
typedef struct prc_compressed_unique_id_s prc_compressed_unique_id;
typedef struct prc_interval_s prc_interval;
typedef struct prc_parameterization_s prc_parameterization;
typedef struct prc_type_asm_modelfile_s prc_type_asm_modelfile;
typedef struct prc_product_occurrence_reference_s prc_product_occurrence_reference;
typedef struct prc_content_layer_filter_item_s prc_content_layer_filter_item;
typedef struct prc_type_mkp_annotation_item_s prc_type_mkp_annotation_item;
typedef struct prc_type_mkp_annotation_set_s prc_type_mkp_annotation_set;
typedef struct prc_type_mkp_annotation_reference_s prc_type_mkp_annotation_reference;
typedef struct prc_topo_multiple_vertex_s prc_topo_multiple_vertex;
typedef struct prc_topo_unique_vertex_s prc_topo_unique_vertex;
typedef struct prc_topo_coedge_s prc_topo_coedge;
typedef struct prc_topo_loop_s prc_topo_loop;
typedef struct prc_coedge_in_loop_s prc_coedge_in_loop;
typedef struct prc_topo_face_s prc_topo_face;
typedef struct prc_topo_shell_s prc_topo_shell;
typedef struct prc_faces_in_shell_s prc_faces_in_shell;
typedef struct prc_topo_connex_s prc_topo_connex;
typedef struct prc_ptr_curve_s prc_ptr_curve;
typedef struct prc_type_surf_s prc_type_surf;
typedef struct prc_ptr_surface_s prc_ptr_surface;
typedef struct prc_topo_s prc_topo;
typedef struct prc_start_end_data_s prc_start_end_data;
typedef struct prc_content_compressed_iso_face_s prc_content_compressed_iso_face;
typedef struct prc_ana_face_trim_loop_s prc_ana_face_trim_loop;
typedef struct prc_content_compressed_ana_face_s prc_content_compressed_ana_face;
typedef struct prc_particular_circle_s prc_particular_circle;
typedef struct prc_general_circle_s prc_general_circle;
typedef struct prc_hcg_line_s prc_hcg_line;
typedef struct prc_hcg_circle_s prc_hcg_circle;
typedef struct prc_iso_nurbs_trim_crv_s prc_iso_nurbs_trim_crv;
typedef struct prc_iso_nurbs_trim_curve_s prc_iso_nurbs_trim_curve;
typedef struct prc_internal_inheritance_s prc_internal_inheritance;
typedef struct prc_internal_detail_inheritance_s prc_internal_detail_inheritance;

/* Abstract Types */
typedef struct prc_tess_s prc_tess;
typedef struct prc_graph_fill_pattern_s prc_graph_fill_pattern;
typedef struct prc_ri_s prc_ri;
typedef struct prc_topo_body_s prc_topo_body;
typedef struct prc_misc_transformation_s prc_misc_transformation;
typedef struct prc_compressed_curve_s prc_compressed_curve;
typedef struct prc_annotation_entity_s prc_annotation_entity;
typedef struct prc_crv_s prc_crv;

/* Others */
typedef struct prc_schema_read_s prc_schema_read;

/* This is an attempt to deal with the behavior bits that deal with inheritence
   along the tree. The Parent inheritence is not handled here and makes little
   sense in the spec. Need to see a file where it occurs. */
struct prc_internal_detail_inheritance_s
{
    uint8_t has_inheritance;
    uint32_t behavior; // Just the ones related to the type (e.g. color, line pattern, etc.)
    uint32_t inherited_file_index;
    uint32_t inherited_value; // Could be an index, or a transparency value
};

struct prc_internal_inheritance_s
{
    uint8_t has_any_inheritance;
    prc_internal_detail_inheritance color;
    prc_internal_detail_inheritance transparency;
    prc_internal_detail_inheritance show;
    prc_internal_detail_inheritance line_pattern;
    prc_internal_detail_inheritance line_width;
    prc_internal_detail_inheritance layer;
};

/* Abstract one */
struct prc_math_fct_1d_s
{
    uint32_t tag;
    union
    {
        prc_math_fct_1d_polynom *polynom;
        prc_math_fct_1d_trigonometric *trigonometric;
        prc_math_fct_1d_fraction *fraction;
        prc_math_fct_1d_arctancos *arctancos;
        prc_math_fct_1d_combination *combination;
    };
};

/* Table 322 — PRC_TYPE_MATH_FCT_1D_Polynom */
struct prc_math_fct_1d_polynom_s
{
    uint32_t tag;
    uint32_t number_of_coefficients;
    double *coefficients;
};

/* Table 323 — PRC_TYPE_MATH_FCT_1D_Trigonometric */
struct prc_math_fct_1d_trigonometric_s
{
    uint32_t tag;
    double amplitude;
    double phase;
    double freq;
    double dc_offset;
};

/* Table 324 — PRC_TYPE_MATH_FCT_1D_Fraction */
struct prc_math_fct_1d_fraction_s
{
    uint32_t tag;
    prc_math_fct_1d numerator;
    prc_math_fct_1d denominator;
};

/* Table 325 — PRC_TYPE_MATH_FCT_1D_ArctanCos */
struct prc_math_fct_1d_arctancos_s
{
    uint32_t tag;
    double a;
    double amplitude;
    double frequency;
    double phase;
    double e;
};

/* Table 327 — CombinationFunctions */
struct prc_combination_functions_s
{
    double coefficient;
    prc_math_fct_1d function;
};

/* Table 326 — PRC_TYPE_MATH_FCT_1D_Combination */
struct prc_math_fct_1d_combination_s
{
    uint32_t tag;
    uint32_t number_of_coefficients;
    prc_combination_functions *coefficients;
};

/* Abstract three-dimensional function */
struct prc_math_fct_3d_s
{
    uint32_t tag;
    union
    {
        prc_math_fct_3d_linear *linear;
        prc_math_fct_3d_nonlinear *nonlinear;
    };
};

struct prc_math_fct_3d_linear_s
{
    uint32_t tag;
    double mat[9];
    double vect[3];
};

struct prc_math_fct_3d_nonlinear_s
{
    uint32_t tag;
    prc_math_fct_3d left_transformation;
    prc_math_fct_3d right_transformation;
    double d2;
    double reserved_double;
    int32_t reserved_int1;
    int32_t reserved_int2;
    int32_t reserved_int3;
};

struct prc_string_s
{
    prc_bool null_flag;
    prc_unsigned_int size;
    unsigned char *string;
};

/* Table 31 */
struct prc_name_s
{
    uint8_t same;
    prc_string name;
};

/* Table 83 - PRC_TYPE_MISC_GeneralTransformation */
struct prc_misc_general_trans_s
{
    prc_unsigned_int tag;
    double general_transform[16];
};

/* Table 87 � 3D Transformation */
struct prc_trans_3d_s
{
    char behavior;  /* TODO is this a char or a uint_32? */
    prc_vec3 translation;
    prc_vec3 non_ortho_matrix[3];
    prc_vec3 rotation[2];
    prc_vec3 non_uniform_scale;
    double scale;
    double homogeneous[4];
};

/* As defined in element of Table 125 (Transformation) which can be MISC_Cartesian or MISC_General */
struct prc_misc_transformation_s
{
    prc_miscellaneous_t transformation_type;
    union
    {
        prc_misc_general_trans general_transformation;  /* An array of 16 doubles */
        prc_trans_3d cartesian;
    };
};

/* 8.5.10 prc_graph_fill_pattern */
struct prc_graph_fill_pattern_s
{
    prc_graphics_t fill_pattern_type;
    union
    {
        prc_graph_dotting_pattern *dotting_pattern;
        prc_graph_hatching_pattern *hatching_pattern;
        prc_graph_solid_pattern *solid_pattern;
        prc_graph_vpicture_pattern *picture_pattern;
    };
};

/* Non-abstract types */
struct prc_bbox_s
{
    prc_vec3 min_corner;
    prc_vec3 max_corner;
};

/* Table 22 */
struct prc_domain_s
{
    prc_vec2 min_uv;
    prc_vec2 max_uv;
};

/* Table 23 */
struct prc_uv_parameterization_s
{
    prc_bool swap_uv;
    prc_domain surface_domain;
    double u_param_coeff_a;
    double v_param_coeff_a;
    double u_param_coeff_b;
    double v_param_coeff_b;
};

/* Table 20 Interval */
struct prc_interval_s
{
    double min_value;
    double max_value;
};

/* Table 21 Parameterization */
struct prc_parameterization_s
{
    prc_interval interval;
    double coeff_a;
    double coeff_b;
};

struct prc_unique_id_s
{
    prc_unsigned_int unique_id0;
    prc_unsigned_int unique_id1;
    prc_unsigned_int unique_id2;
    prc_unsigned_int unique_id3;
};

struct prc_file_struct_desc_s
{
    prc_unique_id unique_id;
    prc_unsigned_int section_count;
    prc_unsigned_int *section_offset;
};

/* Table 11 */
struct prc_uncomp_block_s
{
    prc_unsigned_int block_size;
    uint8_t* block;
    uint8_t* raw_image;
    int x;
    int y;
    int n;
};

/* Table 5 */
struct prc_uncomp_file_s
{
    prc_unsigned_int file_count;
    prc_uncomp_block *files;
};

struct prc_file_structure_header_s
{
    char desc[3];
    prc_unsigned_int min_vers_for_read;
    prc_unsigned_int auth_vers;
    prc_unique_id unique_id_file;
    prc_unique_id unique_id_application;
    prc_unsigned_int file_count;
    prc_uncomp_block *files;  /* This is wrong in the spec */
};

struct prc_entity_schema_s
{
    prc_unsigned_int entity_type;
    prc_unsigned_int token_count;
    prc_unsigned_int *schema_tokens;
};

struct prc_schema_s
{
    prc_unsigned_int schema_count;
    prc_entity_schema* entity_schema;
};

struct prc_header_s
{
    char desc[3];
    prc_unsigned_int min_vers_for_read;
    prc_unsigned_int auth_vers;
    prc_unique_id unique_id_file;
    prc_unique_id unique_id_application;
    prc_unsigned_int filestructure_count;
    prc_file_struct_desc *file_info;
    prc_unsigned_int start_offset;
    prc_unsigned_int end_offset;
    prc_unsigned_int file_count;
    prc_uncomp_file *files;
};

/* Table 30 */
struct prc_attribute_data_s
{
    prc_unsigned_int attribute_count;
    prc_misc_attribute *attributes;
};

/* Table 29 ContentPRCRefBase */
struct prc_content_prc_ref_base_s
{
    prc_attribute_data attribute_data;
    prc_name name;
    prc_unsigned_int nonpersistent_id_cad; /* Missing from current spec! */
    prc_unsigned_int unique_id_cad;
    prc_unsigned_int unique_id;
    uint32_t file_index;
    uint8_t parent_is_RI;
    void *parent;
};

struct prc_content_entity_graphics_reference_s
{
    prc_unsigned_int biased_layer_index;
    prc_unsigned_int biased_index_of_line_style;
    uint8_t behavior_bit_field1;
    uint8_t behavior_bit_field2;
    uint32_t file_index;
};

/* Table 34 GraphicsContent */
struct prc_graphics_content_s
{
    prc_unsigned_int biased_layer_index;
    prc_unsigned_int biased_index_of_line_style;
    uint8_t behavior_bit_field1;
    uint8_t behavior_bit_field2;
    uint8_t has_entity_ref;
    prc_content_entity_graphics_reference entity_ref;
};

/* Table 32 � PRC_TYPE_ROOT_PRCBaseWithGraphics */
struct prc_base_with_graphics_s
{
    prc_content_prc_ref_base base;
    uint8_t same_graphics;
    prc_graphics_content graphics_content;
};

/* Table 73 */
struct prc_attribute_entry_s
{
    prc_bool flag;
    prc_unsigned_int integer_title;
    prc_string string_title;
};

/* Table 74 */
struct prc_attribute_key_value_s
{
    prc_attribute_entry title;
    prc_unsigned_int type;

    union
    {
        prc_unsigned_int value_integer;
        double value_double;
        prc_unsigned_int value_secs_integer;
        prc_string val_string;
    };

    /* These two occur in the same type */
    prc_unsigned_int value_time_msp;
    prc_unsigned_int value_time_lsp;
};

/* Table 16 */
struct prc_compressed_unique_id_s
{
    prc_unsigned_int unique_id0;
    prc_unsigned_int unique_id1;
    prc_unsigned_int unique_id2;
    prc_unsigned_int unique_id3;
};

/* Table 71 */
struct prc_misc_attribute_s
{
    prc_unsigned_int tag;
    prc_attribute_entry attribute_title;
    prc_unsigned_int number_attributes;
    prc_attribute_key_value *attributes;
};

/* Table 28 ContentPRCBase */
struct prc_content_prc_base_s
{
    prc_attribute_data attribute_data;
    prc_name name;
};

/* Table 43 */
struct prc_font_key_s
{
    prc_unsigned_int font_size;
    uint8_t font_attributes;
};

/* Table 42 */
struct prc_font_keys_same_font_s
{
    prc_string font_name;
    prc_unsigned_int character_set;
    prc_unsigned_int key_count;
    prc_font_key* font_key_list;
};

/* Table 41 */
struct prc_markup_serialization_helper_s
{
    prc_string default_font_family_name;
    prc_unsigned_int font_keys_count;
    prc_font_keys_same_font *font_keys_of_font;
};

/* Table 46 */
struct prc_rgb_color_s
{
    double red;
    double green;
    double blue;
    double alpha;
};

/* Table 93 � prc_graph_picture */
struct prc_graph_picture_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    prc_picture_data_format_t format;
    prc_unsigned_int biased_uncompressed_file_index;
    prc_unsigned_int pixel_width;
    prc_unsigned_int pixel_height;
    prc_unsigned_int num_elements_per_pixel;
};

/* Transformations */

/* Table 86 � Transformation type names */
typedef enum {
    PRC_TRANSFORMATION_Identity = 0x00,
    PRC_TRANSFORMATION_Translate = 0x01,
    PRC_TRANSFORMATION_Rotate = 0x02,
    PRC_TRANSFORMATION_Mirror = 0x04,
    PRC_TRANSFORMATION_Scale = 0x08,
    PRC_TRANSFORMATION_NonUniformScale = 0x10,
    PRC_TRANSFORMATION_NonOrtho = 0x20,
    PRC_TRANSFORMATION_Homogeneous = 0x40
} prc_misc_cart_trans_t;

/* Table 76 PRC_TYPE_MISC_CartesianTransformation */
struct prc_cart_transformation_s
{
    prc_misc_cart_trans_t name;
    prc_trans_3d transform;
};

/* Table 88 � 2D Transformation */
struct prc_cartesian_trans_2d_s
{
    uint8_t behavior;
    prc_vec2 translation;
    prc_vec2 non_ortho_matrix[2];
    prc_vec2 rotation[2];
    prc_vec2 non_uniform_scale;
    double scale;
    double homogeneous[3];
};

/* Table 103 PRC_TYPE_GRAPH_TextureTransformation */
struct prc_graph_texture_transformation_s
{
    uint8_t tag;
    uint8_t invert_s;
    uint8_t invert_t;
    uint8_t transform_2d;
    prc_cartesian_trans_2d transform;
};

/* Table 96 � prc_graph_texture_definition */
struct prc_graph_texture_definition_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    prc_unsigned_int biased_picture_index;
    uint8_t texture_dimension;
    PRC_texture_mapping_t texture_mapping_type;
    PRC_texture_mapping_operator_t texture_mapping_operator;
    uint8_t has_transformation;
    prc_cart_transformation transformation;
    PRC_texture_mapping_attributes_t texture_mapping_attributes;
    prc_unsigned_int number_texture_mapping_attributes_intensities;
    double* texture_mapping_attributes_intensities;
    prc_unsigned_int number_of_texture_mapping_attributes_components;
    uint8_t* texture_mapping_attributes_components;
    PRC_texture_function_t texture_function;
    double blend_src[4];
    int32_t blend_src_rgb;
    int32_t blend_src_alpha;
    int32_t blend_des_rgb;
    int32_t blend_des_alpha;
    uint8_t texture_application_mode;
    int32_t alpha_test;
    double alpha_test_reference;
    PRC_texture_wrapping_mode_t texture_wrapping_mode;
    PRC_texture_wrapping_mode_t texture_wrapping_mode_s;
    PRC_texture_wrapping_mode_t texture_wrapping_mode_t;
    PRC_texture_wrapping_mode_t texture_wrapping_mode_r;
    uint8_t has_texture_transformation;
    prc_graph_texture_transformation texture_transformation;
};

/* Table 38.  Oddly named with an enumerated type in the spec */
struct prc_type_asm_file_struct_internal_data_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    prc_unsigned_int next_available_index;
    prc_unsigned_int index_product_occurrence;
};

/* Table 92 � prc_graph_material AND Table 95 PRC_TYPE_GRAPH_TextureApplication
   Turns out both of these types can occur in the materials. There may be more
   and we may want to make an abstract type out of this */
struct prc_graph_material_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;

    /* Valid only if prc_graph_material */
    prc_unsigned_int biased_ambient_index;
    prc_unsigned_int biased_diffuse_index;
    prc_unsigned_int biased_emissive_index;
    prc_unsigned_int biased_specular_index;
    double shininess;
    double ambient_alpha;
    double diffuse_alpha;
    double emissive_alpha;
    double specular_alpha;

    /* Valid only if PRC_TYPE_GRAPH_TextureApplication */
    prc_unsigned_int biased_material_generic_index;
    prc_unsigned_int biased_texture_definition_index;
    prc_unsigned_int biased_next_texture_index;
    prc_unsigned_int biased_uv_coordinates_index;
};

/* Table 104 � prc_graph_line_pattern */
struct prc_graph_line_pattern_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    prc_unsigned_int number_of_elements;
    double* lengths;
    double start_offset;
    uint8_t scale;
};

/* Table 90 prc_graph_style */
struct prc_graph_style_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    double line_width;
    uint8_t is_vpicture;
    prc_unsigned_int biased_pattern_index;
    uint8_t is_material;
    prc_unsigned_int biased_color_index;
    uint8_t is_transparency;
    uint8_t transparency;
    uint8_t is_rendering_parameters;
    uint8_t rendering_parameters;
    uint8_t is_rendering_parameters2; /* Not in the spec */
    uint8_t rendering_parameters2;    /* Not in the spec */
    uint8_t is_rendering_parameters3; /* Not in the spec */
    uint8_t rendering_parameters3;    /* Not in the spec */
};

/* Section 8.8.2 PRC_TYPE_TESS */
struct prc_tess_s
{
    prc_tesslation_t tess_type;
    union
    {
        prc_tess_3d_wire *tess_3d_wire;
        prc_tess_markup *tess_markup;
        prc_tess_3d *tess_3d;
        prc_tess_3d_compressed *tess_3d_compressed;
    };
    size_t num_vertices_internal;
    size_t num_normals_internal;
    double bounding_box_min[3];
    double bounding_box_max[3];
    prc_internal_api_vertex *vertices_internal;
    prc_internal_api_normal *normals_internal;
    prc_internal_api_texture_indices *texture_indices_internal;
    prc_internal_api_position_normal_lookup position_normal_lut;
    uint32_t biased_style_index;
    uint32_t style_file_index;
    uint8_t has_inherited_trans;
    uint8_t has_inherited_color;
    int inherited_trans_index;
    int inherited_color_index;
    uint32_t inherited_color_file_index;
    uint32_t inherited_trans_file_index;
    prc_graph_style default_style; /* Added to make life easier */

    /* We can have an inherited color and transparency, but no style (e.g.
       the glass in the LED flashlight) */
};

/* Table 137 � ContentBaseTessData */
struct prc_content_base_tess_data_s
{
    uint8_t is_calculated;
    prc_unsigned_int number_of_coordinates;
    double *coordinates;
};

/* A helper struct for getting data organized into primitives */
typedef enum {
    MARKUP_LINE = 3,
    MARKUP_LINE_STRIP = 4,
    MARKUP_LINE_LOOP = 5,
    MARKUP_TRIANGLE = 6,
    MARKUP_TEXT = 7
} prc_markup_primitive_t;

typedef struct prc_markup_primitive_decode_s
{
    uint32_t number_indices;
    uint32_t *indices;
    prc_markup_primitive_t primitive_type;
    uint32_t biased_color_index;
    uint32_t biased_font_index;
    char *text;
    double text_width;
    double text_height;
    double line_width;
    prc_markup_block_mode_t block_mode;
    prc_vec3 face_frame_draw_origin;
} prc_markup_primitive_decode;

/* Table 147 � prc_tess_markup */
struct prc_tess_markup_s
{
    prc_unsigned_int tag;
    prc_content_base_tess_data tessellation_coordinates;
    prc_unsigned_int number_of_codes;
    prc_unsigned_int* code_numbers;
    prc_unsigned_int number_of_text_strings;
    prc_string* text_strings;
    prc_string tessellation_label;
    uint8_t behavior;

    /* Decoded data */
    uint32_t decode_num_vertices;
    prc_vec3 *decode_vertices;
    uint32_t decode_number_primitives;
    prc_markup_primitive_decode *decode_primitives;
};

/* Table 105 � PRC_TYPE_GRAPH_DottingPattern */
struct prc_graph_dotting_pattern_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    prc_unsigned_int biased_next_pattern_index;
    double pitch;
    uint8_t is_offset;
    int32_t biased_color_index;
};

struct prc_hatch_s
{
    double startpoint_x;
    double startpoint_y;
    double endpoint_x;
    double endpoint_y;
    double angle;
    int32_t style_index;
};

/* Table 106 � PRC_TYPE_GRAPH_HatchingPattern */
struct prc_graph_hatching_pattern_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    prc_unsigned_int biased_next_pattern_index;
    prc_unsigned_int number_of_hatching_lines;
    prc_hatch* hatch;
};

/* Table 107 � PRC_TYPE_GRAPH_SolidPattern */
struct prc_graph_solid_pattern_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    prc_unsigned_int biased_next_pattern_index;
    uint8_t is_material;
    prc_unsigned_int biased_material_index;
    prc_unsigned_int biased_color_index;
};

/* Table 108 � PRC_TYPE_GRAPH_VpicturePattern */
struct prc_graph_vpicture_pattern_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    prc_unsigned_int biased_next_pattern_index;
    double pattern_dimensions[2];
    uint8_t is_material;
    prc_unsigned_int biased_material_index;
    prc_tess_markup markup;
};

/* Aliased Transformation */
struct prc_aliased_transform_s
{
    prc_unsigned_int tag;
    void* data;
    PRC_cartesian_transformation_name_t name;
};

/* Table 18 UserData */
struct prc_user_data_s
{
    prc_unsigned_int stream_size;
    void* user_data;
};

/* Table 116 */
struct prc_representation_item_content_s
{
    prc_base_with_graphics base;
    prc_unsigned_int biased_index_local_coordinate_system;
    prc_unsigned_int biased_index_tessellation;

    /* This bit is used for inheriting properties from a parent */
    prc_internal_inheritance style_inheritance;
};

/* Table 40 � FileStructureInternalGlobalData */
struct prc_file_struct_internal_global_data_s
{
    double tess_chord;
    double tess_angle;
    prc_markup_serialization_helper serialize_help;
    prc_unsigned_int color_count;
    prc_rgb_color *colors;
    prc_unsigned_int picture_count;
    prc_graph_picture *pictures;
    prc_unsigned_int texture_count;
    prc_graph_texture_definition *textures;
    prc_unsigned_int material_count;
    prc_graph_material *materials;
    prc_unsigned_int line_pattern_count;
    prc_graph_line_pattern *line_patterns;
    prc_unsigned_int style_count;
    prc_graph_style *styles;
    prc_unsigned_int fill_count;
    prc_graph_fill_pattern *fills;
    prc_unsigned_int ref_coord_count;
    prc_ri_coordinate_system *ref_coords;
};

/* Table 26 BoundingBox */
struct prc_bounding_box_s
{
    prc_vec3 minimum_corner;
    prc_vec3 maximum_corner;
};

/* Table 82 � AdditionalTargetData */
struct prc_additional_target_data_s
{
    uint8_t flag;
    prc_unique_id unique_id;
    prc_unsigned_int index_of_topological_index;
    prc_unsigned_int index_of_body;
    prc_unsigned_int number_of_indices;
    prc_unsigned_int *indices;
};

/* Table 81 � PRC_TYPE_MISC_ReferenceOnTopology */
struct prc_misc_reference_on_topology_s
{
    prc_unsigned_int tag;
    prc_unsigned_int type;
    uint8_t flag;
    prc_additional_target_data data;
};

/* Table 80 � PRC_TYPE_MISC_ReferenceOnPRCBase */
struct prc_misc_reference_on_prcbase_s
{
    prc_unsigned_int tag;
    prc_unsigned_int type_of_entity;
    uint8_t flag;
    prc_unique_id different_unique_id;
    prc_unsigned_int unique_id;
};

/* Table 85 � ReferenceData */
struct prc_reference_data_s
{
    prc_unsigned_int ref_type;  /* Redundant but identifies the type we have */
    union {
        prc_misc_reference_on_topology topo_reference;
        prc_misc_reference_on_prcbase non_topo_reference;
    };
};

/* Table 84 � ContentEntityReference */
struct prc_content_entity_reference_s
{
    prc_base_with_graphics base;
    prc_unsigned_int index_of_local_coordinate;
    prc_bool flag;
    prc_reference_data reference_data;
};

/* Table 79 � ContentExtendedEntityReference */
struct prc_content_extended_entity_ref_s
{
    prc_content_entity_reference content_entity_ref;
    uint8_t has_reference_data; /* Missing in original spec */
    prc_reference_data reference_data;
};

/* Table 78 � prc_misc_markup_linked_item */
struct prc_misc_markup_linked_item_s
{
    prc_unsigned_int tag;
    prc_content_extended_entity_ref content_entity_ref;
    uint8_t show_markup;
    uint8_t delete_markup;
    uint8_t show_leader;
    uint8_t delete_leader;
    prc_user_data user_data;
};

/* Table 132 � prc_mkp_leader */
struct prc_mkp_leader_s
{
    prc_unsigned_int tag;
    prc_base_with_graphics base;
    uint8_t is_first_linked_item;
    prc_misc_reference_on_prcbase first_linked_item;
    uint8_t is_second_linked_item;
    prc_misc_reference_on_prcbase second_linked_item;
    prc_unsigned_int biased_index_tessallation;
    prc_user_data user_data;
};

/* Table 131 � prc_mkp_markup */
struct prc_mkp_markup_s
{
    prc_unsigned_int tag;
    prc_base_with_graphics base;
    prc_unsigned_int markup_type;
    prc_unsigned_int markup_subtype;
    prc_unsigned_int number_of_linked_items;
    prc_misc_reference_on_prcbase *linked_items;
    prc_unsigned_int number_of_leaders;
    prc_misc_reference_on_prcbase *leaders;
    prc_unsigned_int biased_index_tessellation;
    prc_user_data user_data;
};

/* Table 135 */
struct prc_type_mkp_annotation_reference_s
{
    prc_unsigned_int tag;
    prc_base_with_graphics base;
    prc_unsigned_int number_of_linked_items;
    prc_misc_reference_on_prcbase *linked_items;
    prc_user_data user_data;
};

/* Table 134 */
struct prc_type_mkp_annotation_set_s
{
    prc_unsigned_int tag;
    prc_base_with_graphics base;
    prc_unsigned_int number_of_annotations;
    prc_annotation_entity *annotations;
    prc_user_data user_data;
};

/* Table 128 ReferenceUniqueIdentifier */
struct prc_reference_unique_identifiers_s
{
    prc_unsigned_int tag;
    prc_unsigned_int type;
    uint8_t reference_in_same_file_structure;
    prc_unique_id target_file_structure;
    prc_unsigned_int unique_id;
};

/* Table 133 */
struct prc_type_mkp_annotation_item_s
{
    prc_unsigned_int tag;
    prc_base_with_graphics base;
    prc_misc_reference_on_prcbase unique_id;
    prc_user_data user_data;
};

/* Seciont 8.3.10.5 AnnotationEntities Abstract type */
struct prc_annotation_entries_s
{
    prc_unsigned_int tag;
    union
    {
        prc_type_mkp_annotation_item annotation_item;
        prc_type_mkp_annotation_set annotation_set;
        prc_type_mkp_annotation_reference annotation_ref;
    };
};

/* Table 65 � MarkupData */
struct prc_markup_data_s
{
    prc_unsigned_int number_of_linked_items;
    prc_misc_markup_linked_item *linked_items;
    prc_unsigned_int number_of_leaders;
    prc_mkp_leader *leaders;
    prc_unsigned_int number_of_markups;
    prc_mkp_markup *markups;
    prc_unsigned_int number_of_annotation_entities;
    prc_annotation_entity *annotation_entities;
};

/* Table 77 � PRC_TYPE_MISC_EntityReference */
struct prc_misc_entity_reference_s
{
    prc_unsigned_int tag;
    prc_content_entity_reference content_entity_ref;
    prc_user_data user_data;
};

/* Table 69 � ContentEntityFilterItems */
struct prc_content_entity_filter_item_s
{
    uint8_t b_is_inclusive;
    prc_unsigned_int number_of_entities;
    prc_misc_entity_reference* entities;
};

/* Table 68 */
struct prc_content_layer_filter_item_s
{
    uint8_t b_is_inclusive;
    prc_unsigned_int number_of_layers;
    prc_unsigned_int *layers;
};

/* Table 67 � prc_asm_filter */
struct prc_asm_filter_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    uint8_t is_active;
    prc_content_layer_filter_item layer_filter;
    prc_content_entity_filter_item entity_filter;
    prc_user_data user_data;
};

/* Table 285 � ContentSurface */
struct prc_content_surface_s
{
    uint8_t has_base_geometry;
    prc_attribute_data attribute_data;
    prc_name name;
    prc_unsigned_int id;
    prc_unsigned_int extension_type;
};

/* 8.5.15 PRC_TYPE_GRAPH_AmbientLight Abtract type  */
/* Table 109 Table 110 Table 111 Table 112 */
struct prc_graph_light_object_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    prc_unsigned_int biased_ambiant_index;
    prc_unsigned_int biased_diffuse_index;
    prc_unsigned_int biased_emmissive_index;
    prc_unsigned_int biased_specular_index;
    prc_vec3 location; /* point light */
    double constant_attenuation_factor; /* point light */
    double linear_attenuation_factor; /* point light */
    double quadratic_attenuation_factor; /* point light */
    prc_vec3 direction; /* directional light */
    double intensity; /* directional light */
    double fall_off_angle;
    double fall_off_exponenent;
};

/* Table 114 � prc_graph_camera */
struct prc_graph_camera_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    uint8_t is_orthographic;
    prc_vec3 position;
    prc_vec3 look;
    prc_vec3 up;
    double x;
    double y;
    double ratio;
    double clip_near;
    double clip_far;
    double zoom;
};

/* Table 113 � PRC_TYPE_GRAPH_SceneDisplayParameters */
struct prc_scene_display_parameters_s
{
    prc_unsigned_int tag;
    prc_content_prc_ref_base base;
    uint8_t is_active;
    prc_unsigned_int number_of_lights;
    prc_graph_light_object* lights;
    uint8_t camera_defined;
    prc_graph_camera camera;
    uint8_t rotation_center_defined;
    prc_vec3 rotation_center;
    prc_unsigned_int number_of_clipping_planes;
    prc_surf_plane* clipping_planes;  /* or is this a  Surface entity type */
    prc_unsigned_int index_of_line_style_background;
    prc_unsigned_int index_of_line_style_default;
    prc_unsigned_int number_default_styles;
    prc_unsigned_int* styles;
    uint8_t is_absolute;
};

/* Table 306 — PRC_TYPE_SURF_Plane */
struct prc_surf_plane_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    prc_trans_3d transform; /* Does not have a has_transform bit */
    prc_domain domain;
    double u_parameter_coeff_a;
    double v_parameter_coeff_a;
    double u_parameter_coeff_b;
    double v_parameter_coeff_b;
};

/* Table 127 � prc_mkp_view */
struct prc_mkp_view_s
{
    prc_unsigned_int tag;
    prc_base_with_graphics base;
    prc_unsigned_int number_of_annotations;
    prc_misc_reference_on_prcbase* annotations;
    prc_surf_plane annotation_plane;  /* or is this a  Surface entity type */
    uint8_t has_parameters;
    prc_scene_display_parameters scene_display_parameters;
    uint8_t is_annotation_view;
    uint8_t is_default_view;
    uint8_t is_direction;
    prc_unsigned_int number_of_linked_items;
    prc_misc_reference_on_prcbase* linked_items;
    prc_unsigned_int number_of_filters;
    prc_asm_filter* filters;
    prc_user_data user_data;
};

/* Table 66 � PRC_TYPE_ASM_PartDefinition */
struct prc_asm_parts_definition_s
{
    prc_unsigned_int tag;
    prc_base_with_graphics base;
    prc_bounding_box bounding_box;
    prc_unsigned_int num_rep_items;
    prc_ri *rep_items;
    prc_markup_data markups;
    prc_unsigned_int number_views;
    prc_mkp_view *views;
    prc_user_data user_data;

    /* This bit is used for inheriting properties from a parent */
    prc_internal_inheritance style_inheritance;
};

/* Table 61 �FileIdentifier */
struct prc_file_identifier_s
{
    prc_bool flag;
    prc_unique_id unique_id;
};

/* Table 60 � ReferencesOfProductOccurrence */
struct prc_references_of_product_occurrence_s
{
    prc_unsigned_int biased_index_part;
    prc_unsigned_int biased_index_prototype;
    prc_file_identifier prototype_in_same_file_structure;
    prc_unsigned_int biased_index_external_data;
    prc_file_identifier external_data_in_same_file_structure;
    prc_unsigned_int number_of_child_product_occurrences;
    prc_unsigned_int *index_child_occurrence;
};

/* Table 37 */
struct prc_product_occurrence_reference_s
{
    prc_unique_id unique_id;
    prc_unsigned_int root_index;
    uint8_t product_occurence_is_active;
};

/* Table 36 */
struct prc_type_asm_modelfile_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    uint8_t units_from_CAD_flag;
    double units_from_CAD_file;
    prc_unsigned_int number_of_root_product_occurrences;
    prc_product_occurrence_reference *product_occurences;
    prc_unsigned_int *file_structure_index_in_model_file;
    prc_user_data user_data;
};

/* Table 62 ProductInformation */
struct prc_product_information_s
{
    prc_bool unit_from_CAD_file;
    double unit;
    uint8_t product_information_flags;
    int32_t product_load_status;
};

/* Table 59 PRC_TYPE_ASM_ProductOccurrence */
struct prc_asm_product_occurrence_s
{
    prc_unsigned_int tag;
    prc_base_with_graphics base;
    prc_references_of_product_occurrence references_product_occurrence;
    uint8_t product_behavior;
    prc_product_information product_information;
    prc_bool has_transform;
    prc_misc_transformation location;
    prc_unsigned_int entity_ref_count;
    prc_misc_entity_reference *entity_reference;
    prc_markup_data markups;
    prc_unsigned_int number_of_views;
    prc_mkp_view *views;
    prc_bool has_filter;
    prc_asm_filter entity_filter;
    prc_unsigned_int number_of_display_filters;
    prc_asm_filter *display_filters;
    prc_unsigned_int number_of_scene_parameters;
    prc_scene_display_parameters *scene_display_parameters;
    prc_user_data user_data;

    /* This bit is used for inheriting properties from a parent */
    prc_internal_inheritance style_inheritance;
};

/* Table 145 ColorDataRemainder */
struct prc_color_data_remainder_s
{
    prc_bool is_same;
    prc_rgb_color color;
};

/* Table 144 ColorData */
struct prc_color_data_s
{
    prc_rgb_color first_vertex;
    prc_color_data_remainder *remaining_vertices;
};

/* Table 143 VertexColors */
struct prc_vertex_colors_s
{
    prc_bool is_rgba;
    prc_bool is_segment_color;
    prc_bool b_optimized;
    prc_color_data color_data;
};

struct prc_tess_3d_wire_element_s
{
    prc_unsigned_int number_of_wire_indexes;
    prc_unsigned_int *wire_indexes;
    uint32_t is_connected;
};

/* Table 142 PRC_TYPE_TESS_3D_Wire */
struct prc_tess_3d_wire_s
{
    prc_unsigned_int tag;
    prc_content_base_tess_data tessellation_coordinates;
    prc_unsigned_int number_of_wire_indexes_encoded;
    prc_unsigned_int number_of_wire_indexes;
    uint32_t number_of_wire_elements;
    prc_tess_3d_wire_element *wire_elements;
    prc_bool has_vertex_colors;
    uint32_t vertex_color_count;
    prc_vertex_colors vertex_color_data;
};

/* Table 140 PRC_TYPE_TESS_Face */
struct prc_tess_face_s
{
    prc_unsigned_int tag;
    prc_unsigned_int size_of_line_attributes;
    prc_unsigned_int *line_attributes;
    prc_unsigned_int start_of_wire_data;
    prc_unsigned_int size_of_sizes_wire;
    prc_unsigned_int *sizes_wire;
    prc_unsigned_int used_entities_flag;
    prc_unsigned_int start_triangulated;
    prc_unsigned_int size_of_triangulateddata;
    prc_unsigned_int *triangulateddata;
    prc_unsigned_int number_of_textured_coordinate_indexes;
    prc_bool has_vertex_colors;
    prc_vertex_colors vertex_colors;
    prc_unsigned_int behavior;
};

/* Table 138 PRC_TYPE_TESS_3D */
struct prc_tess_3d_s
{
    prc_unsigned_int tag;
    prc_content_base_tess_data tessellation_coordinates;
    prc_bool has_faces;
    prc_bool has_loops;
    prc_bool must_calculate_normals;
    uint8_t normal_recalculation_flags;
    double crease_angle;
    prc_unsigned_int number_of_normal_coordinates;
    double* normal_coordinates;
    prc_unsigned_int number_of_wire_indices;
    prc_unsigned_int *wire_indices;
    prc_unsigned_int number_of_triangulated_indicies;
    prc_unsigned_int *triangulated_index_array;
    prc_unsigned_int number_of_face_tessellation;
    prc_tess_face *face_tessellation_data;
    prc_unsigned_int number_of_texture_coordinates;
    double* texture_coordinates;
};

/* Table 176 PRC_TYPE_BINARY_TEXTURE_DATA */
struct prc_binary_texture_data_s
{
    prc_unsigned_int texture_binary_data_size;
    uint8_t *texture_binary_data;
    prc_unsigned_int last_integer_used_bit_number;
};

/* Table 175 PRC_TYPE_COMPRESSED_TEXTURE_PARAMETER */
struct prc_compressed_texture_parameter_s
{
    prc_binary_texture_data *binary_texture_data;
    uint32_t reference_array_size;
    uint32_t *reference_array;
    double texture_parameters_tolerance;
    uint32_t texture_parameters_size;
    float *texture_parameters;
};

/* Table 174 PRC_TYPE_TESS_3D_COMPRESSED Description of the Data Written to the File */
struct prc_tess_3d_compressed_s
{
    prc_unsigned_int tag;
    prc_bool is_calculated;
    prc_bool has_faces;
    double tolerance;
    prc_vec3 origin_array;
    uint32_t point_array_size;
    int32_t *point_array;
    prc_vec3 *point_array_uncompressed;
    uint32_t edge_status_array_size;
    uint8_t *edge_status_array;
    uint32_t triangle_face_array_size;
    uint32_t *triangle_face_array;
    uint32_t face_number;
    prc_unsigned_int reference_array_size;
    prc_bool *points_is_reference_array;
    uint32_t point_reference_array_size;
    int32_t *point_reference_array;
    prc_bool must_recalculate_normals;
    prc_bool *normal_is_reversed;
    double crease_angle;
    uint8_t normal_recalculation_flags;
    uint8_t normal_angle_number_of_bits;
    prc_unsigned_int normal_binary_data_size;
    prc_bool *normal_binary_data;
    int32_t *normal_angle_array;
    uint32_t normal_angle_array_size;
    double *normal_angle_array_scaled;
    prc_bool *is_face_planar;
    prc_bool is_point_color;
    prc_bool *is_point_color_on_face;
    uint32_t point_color_array_size;
    uint8_t *point_color_array;
    uint32_t decoded_point_color_array_size;
    float *decoded_point_color_array;
    prc_bool is_multiple_line_attribute;
    prc_bool *is_multiple_line_attribute_on_face;
    uint32_t line_attribute_array_size;
    int32_t *line_attribute_array;
    prc_bool no_texture;
    prc_compressed_texture_parameter *texture_data;
    prc_bool all_faces_have_texture;
    prc_bool *face_has_texture;
    prc_bool has_behaviors;
    uint32_t behaviors_array_size;
    uint8_t *behaviors_array;

    /* Uncompressed data */
    float *uv_coordinates_3d;  /* 2D texture coordinates */
    int num_triangle_indices_prc_compressed_3d;
    uint32_t *triangle_indices_prc_compressed_3d;
    int num_normal_indices_prc_compressed_3d;
    uint32_t *normal_indices_prc_compressed_3d;
    int num_vertices_prc_compressed_3d;
    double *vertices_prc_compressed_3d;
    int num_normals_prc_compressed_3d;
    double *normals_prc_compressed_3d;
    float *point_colors_prc_compressed_3d; /* Vertex colors if included (point_colors) */
	uint32_t *triangle_styles; /* Line styles */

    /* Structure to hold line edges */
    uint32_t number_of_edges;
    double *edge_vertices; /* 2 vertices per edge */
    uint32_t *edge_indices; /* 2 vertex indices per edge */
};

/* Table 48 PRC_TYPE_ASM_FileStructureTessellation */
struct prc_asm_file_structure_tessellation_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    prc_unsigned_int tess_count;
    prc_tess *tess;
    prc_user_data user_data;
};

/* Table 47 PRC_TYPE_ASM_FileStructureTree */
struct prc_asm_file_structure_tree_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    prc_unsigned_int parts_count;
    prc_asm_parts_definition *parts;
    prc_unsigned_int product_count;
    prc_asm_product_occurrence *products;
    prc_type_asm_file_struct_internal_data internal_data;
    prc_user_data user_data;
};

/* Table 39 PRC_TYPE_ASM_FileStructureGlobals */
struct prc_asm_file_structure_globals_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    prc_unsigned_int file_count;
    prc_unique_id *unique_ids;
    prc_file_struct_internal_global_data global_data;
    prc_user_data user_data;
};

/* Table 246 — ContentCurve */
struct prc_content_curve_s
{
    uint8_t has_base_geometry;
    prc_attribute_data attribute_data;
    prc_name name;
    prc_unsigned_int id;
    prc_unsigned_int extension_type;
    uint8_t is_3d_flag;
};

/* Table 50 FileStructureExactGeometry */
struct prc_file_structure_exact_geometry_s
{
    prc_unsigned_int topo_context_count;
    prc_topo_context *topo_contexts;  /* Defined in 8.9.4 of spec Table 180.  It is related to the BREP (boundary representation) */
};

/* Table 54 Body Information */
struct prc_body_information_s
{
    prc_unsigned_int body_serial_type;
    double tolerance;
};

/* Table 58 Element Graphics Behavior */
struct prc_element_graphics_behavior_s
{
    uint8_t use_context;
    prc_unsigned_int biased_layer_index;
    prc_unsigned_int biased_index_of_line_style;
    uint8_t behavior_bit_field[2];
};

/* Table 57 Element Information */
struct prc_element_information_s
{
    uint8_t has_graphics;
    prc_element_graphics_behavior graphic_behavoir;
};

/* Table 56 � Graphics Information */
struct prc_graphics_information_s
{
    prc_unsigned_int element_type;
    prc_unsigned_int number_of_element;
    prc_element_information *element_information;
};

/* Table 55 � Context Graphics */
struct prc_context_graphics_s
{
    prc_unsigned_int number_of_treat_type;
    prc_graphics_information *treat_types;
};

/* Table 53 � Geometry Summary */
struct prc_geometry_summary_s
{
    prc_unsigned_int number_of_bodies;
    prc_body_information *bodies;
};

/* Table 52 � Extra Geometry (mislabled in spec as ExactGeometry argh) and not to be confused with FileStructureExactGeometry */
struct prc_extra_geometry_s
{
    prc_geometry_summary summary;
    prc_context_graphics context_graphics;
};

/* Table 51 � PRC_TYPE_ASM_FileStructureExtraGeometry */
struct prc_asm_file_structure_extra_geometry_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    uint32_t extra_geom_count;
    prc_extra_geometry *extra_geom;
    prc_user_data user_data;
};

/* Table 49 � PRC_TYPE_ASM_FileStructureGeometry */
struct prc_asm_file_structure_geometry_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    prc_file_structure_exact_geometry exact_geometry;
    prc_user_data user_data;
};

/* Table 6 � Filestructure */
struct prc_filestructure_s
{
    prc_file_structure_header* header;

    uint8_t* schema_globals_unzipped;
    uint8_t* tree_unzipped;
    uint8_t* tessellation_unzipped;
    uint8_t* geometry_unzipped;
    uint8_t* extra_geometry_unzipped;
    uint8_t *model_unzipped;

    size_t schema_globals_size;
    size_t tree_size;
    size_t tessellation_size;
    size_t geometry_size;
    size_t extra_geometry_size;
    size_t model_size;

    prc_asm_file_structure_globals* globals;
    prc_asm_file_structure_tree *tree;
    prc_asm_file_structure_tessellation *tessellation;
    prc_asm_file_structure_geometry *geometry;
    prc_asm_file_structure_extra_geometry *extra_geometry;
    prc_type_asm_modelfile *model;
};

/* Table 239 (241) Compressed Point */
struct prc_compressed_point_s
{
    prc_unsigned_int uNbBits; /* Note variable bit number (stored with 6 bits is says) */
    prc_vec3 point; /* Note this can be stored multiple ways */
};

/* Table 239 Compressed Vertex */
struct prc_compressed_vertex_s
{
    uint8_t not_already_stored;
    prc_unsigned_int point_index; /* Note variable bit number */
    prc_compressed_point point_data;
};

/* Table 178 BaseTopology */
struct prc_base_topology_s
{
    uint8_t has_base;
    prc_attribute_data attribute_data;
    prc_name name;
    prc_unsigned_int id;
};

/* Table 193 ContentBody */
struct prc_content_body_s
{
    prc_base_topology base_topology;
    uint8_t bounding_box_behavior;
};

/* Table 243 PtrTopology */
struct prc_ptr_topology_s
{
    uint8_t is_stored;
    prc_topo *topo;
    prc_unsigned_int topo_identifier;
};

/* Table 196 PRC_TYPE_TOPO_BrepData */
struct prc_topo_brep_data_s
{
    prc_unsigned_int tag;
    prc_content_body base;
    prc_unsigned_int number_of_connex;
    prc_ptr_topology *connex;
    prc_bounding_box bounding_box;
};

/* Abstract types */
/* Table 242 — PtrCurve */
/* All the curves */
struct prc_ptr_curve_s
{
    uint8_t is_referenced;
    uint32_t curve_type;
    union
    {
        prc_crv_blend02_boundary *crv_blend02_boundary;
        prc_crv_nurbs *crv_nurbs;
        prc_crv_circle *crv_circle;
        prc_crv_composite *crv_composite;
        prc_crv_onsurf *crv_onsurf;
        prc_crv_ellipse *crv_ellipse;
        prc_crv_equation *crv_equation;
        prc_crv_helix01 *crv_helix01;
        prc_crv_hyperbola *crv_hyperbola;
        prc_crv_intersection *crv_intersection;
        prc_crv_line *crv_line;
        prc_crv_offset *crv_offset;
        prc_crv_parabola *crv_parabola;
        prc_crv_polyline *crv_polyline;
        prc_crv_transform *crv_transform;
    };
    prc_unsigned_int curve_identifier;
};

/* Just the surface which is needed for AnaGenericFace */
struct prc_type_surf_s
{
    uint32_t surface_type;
    union
    {
        prc_surf_blend01 *surf_blend01;
        prc_surf_blend02 *surf_blend02;
        prc_surf_blend03 *surf_blend03;
        prc_surf_nurbs *surf_nurbs;
        prc_surf_cone *surf_cone;
        prc_surf_cylinder *surf_cylinder;
        prc_surf_cylindrical *surf_cylindrical;
        prc_surf_offset *surf_offset;
        prc_surf_pipe *surf_pipe;
        prc_surf_plane *surf_plane;
        prc_surf_ruled *surf_ruled;
        prc_surf_sphere *surf_sphere;
        prc_surf_revolution *surf_revolution;
        prc_surf_extrusion *surf_extrusion;
        prc_surf_fromcurves *surf_fromcurves;
        prc_surf_torus *surf_torus;
        prc_surf_transform *surf_transform;
    };
};

/* Table 243 — PtrSurface */
/* All the surfaces */
struct prc_ptr_surface_s
{
    uint8_t is_referenced;
    prc_type_surf surface;
    prc_unsigned_int surface_identifier;
};

/* Table 288 — PRC_TYPE_SURF_Blend01 */
struct prc_surf_blend01_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_ptr_curve center_curve;
    prc_ptr_curve origin_curve;
    prc_ptr_curve tangent_curve;
};

/* Table 290 — PRC_TYPE_SURF_Blend02 */
struct prc_surf_blend02_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_ptr_surface bound_surface0;
    prc_ptr_curve bound_curve0;
    prc_ptr_surface bound_surface1;
    prc_ptr_curve bound_curve1;
    prc_ptr_curve center_curve;
    uint8_t center_curve_sense;
    uint8_t bound_surface0_sense;
    uint8_t bound_surface1_sense;
    double radius0;
    double radius1;
    prc_ptr_surface cliff_surface0;
    prc_ptr_surface cliff_surface1;
    uint8_t parameterization_type;
};

/* Table 292 — PRC_TYPE_SURF_Blend03 */
struct prc_surf_blend03_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    uint32_t number_of_elements;
    double *parameters;
    int32_t *multiplicities;
    prc_vec3 *points;
    double *rail_2_angles_v;
    prc_vec3 *tangents;
    double *rail_2_derivatives_v;
    prc_vec3 *second_derivatives;
    double *rail_2_second_derivatives;
    double rail_2_parameter_v;
    double trim_v_min;
    double trim_v_max;
    int32_t reserved_int[6];
    uint8_t reserved_char_0;
    uint8_t reserved_char_1;
    uint8_t reserved_char_2;
    int32_t number_of_supplemental_doubles;
    double *supplemental_doubles;
};

struct prc_control_points_nurbs_surf_s
{
    double x;
    double y;
    double z;
    double w; /* Weight - only used if is_rational is true */
};

struct prc_surf_nurbs_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t is_rational;
    uint32_t du;
    uint32_t dv;
    uint32_t highest_index_of_control_points_u;
    uint32_t highest_index_of_control_points_v;
    uint32_t highest_index_of_knots_u;
    uint32_t highest_index_of_knots_v;
    prc_control_points_nurbs_surf *p; /* Control points - array of size (highest_index_of_control_points_u+1)*(highest_index_of_control_points_v+1) */
    double *knot_vector_u; /* Array of size (highest_index_of_knots_u+1) */
    double *knot_vector_v; /* Array of size (highest_index_of_knots_v+1) */
    uint32_t knot_type;
    uint32_t surface_form;
};
/* Table 297 — PRC_TYPE_SURF_Cone */
struct prc_surf_cone_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    double radius;
    double semi_angle;
};

/* Table 299 — PRC_TYPE_SURF_Cone */
struct prc_surf_cylinder_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    double radius;
};

/* Table 301 — PRC_TYPE_SURF_Cylindrical */
struct prc_surf_cylindrical_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_ptr_surface base_surface;
    double tolerance;
};

/* Table 303 — PRC_TYPE_SURF_Offset */
struct prc_surf_offset_s
{   prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_ptr_surface base_surface;
    double offset_distance;
};

/* Table 304 — PRC_TYPE_SURF_Pipe */
struct pr_surf_pipe_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_ptr_curve center_curve;
    prc_ptr_curve origin_curve;
    double radius;
};

/* Table 308 — PRC_TYPE_SURF_Ruled */
struct prc_surf_ruled_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_ptr_curve first_curve;
    prc_ptr_curve second_curve;
};

/* Table 310 — PRC_TYPE_SURF_Sphere */
struct prc_surf_sphere_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    double radius;
};

/*Table 312 — PRC_TYPE_SURF_Revolution */
struct prc_surf_revolution_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    double tolerance;
    prc_vec3 origin;
    prc_vec3 x_axis;
    prc_vec3 y_axis;
    prc_ptr_curve base_curve;
};

/* Table 314 — PRC_TYPE_SURF_Extrusion */
struct prc_surf_extrusion_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_vec3 sweep_vector;
    prc_ptr_curve base_curve;
};

/*Table 316 — PRC_TYPE_SURF_FromCurves */
struct prc_surf_fromcurves_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_vec3 origin;
    prc_ptr_curve first_curve;
    prc_ptr_curve second_curve;
};

/* Table 318 — PRC_TYPE_SURF_Torus */
struct prc_surf_torus_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    double major_radius;
    double minor_radius;
};

/* Table 320 — PRC_TYPE_SURF_Transform */
struct prc_surf_transform_s
{
    prc_unsigned_int tag;
    prc_content_surface curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_uv_parameterization parameterization;
    prc_ptr_surface base_surface;
    prc_math_fct_3d math_transform;
};

/* Table 248 — PRC_TYPE_CRV_Blend02Boundary */
struct prc_crv_blend02_boundary_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    uint8_t has_transform; /* This is not in the spec */
    prc_trans_3d transform;
    prc_parameterization parameterization;
    prc_ptr_surface surface;
    int bound;
    uint32_t number_of_crossing_points;
    prc_vec3 *crossing_points;
    double chord_error;
    double angle_error;
    prc_ptr_surface bound_surface;
    uint8_t sense_bound_surface;
    uint8_t intersection_order;
    uint8_t sense_intersection_order;
    double base_parameter;
    double base_scale;
    prc_vec3 start_limit_point;
    uint32_t start_limit_type;
    prc_vec3 end_limit_point;
    uint32_t end_limit_type;
};

/* Table 250 — ControlPointsNURBSCrv */
struct prc_control_points_nurbs_crv_s
{
    double x;
    double y;
    double z;
    double w;
};

/* Table 249 — PRC_TYPE_CRV_NURBS */
struct prc_crv_nurbs_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    uint8_t is_rational;
    uint32_t d;
    uint32_t highest_index_of_control_points;
    uint32_t highest_index_of_knots;
    prc_control_points_nurbs_crv *p;
    double *u;
    uint32_t knot_type;
    uint32_t curve_form;
};

/* Table 255 — PRC_TYPE_CRV_Circle */
struct prc_crv_circle_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    uint8_t has_transform;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    double radius;
};

/* Table 258 — CompositeSubCurve */
struct prc_composite_subcurve_s
{
    prc_ptr_curve ptr_curve;
    uint8_t sense;
};

/* Table 257 — PRC_TYPE_CRV_Composite */
struct prc_crv_composite_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    prc_unsigned_int number_of_subcurves;
    prc_composite_subcurve *subcurves;
    uint8_t is_closed;
};

/* Table 260 — PRC_TYPE_CRV_OnSurf */
struct prc_crv_onsurf_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    double tolerance;
    prc_ptr_curve uv_curve;
    prc_ptr_surface surface;
    prc_domain uv_domain;
};

/* Table 262 — PRC_TYPE_CRV_Ellipse */
struct prc_crv_ellipse_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    uint8_t has_transform;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    double rx;
    double ry;
};

/* Table 264 — PRC_TYPE_CRV_Equation */
struct prc_crv_equation_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    prc_interval interval;
    prc_math_fct_1d x_function;
    prc_math_fct_1d y_function;
    prc_math_fct_1d z_function;
};

/* Table 267 — Type0HelixData */
struct prc_type0_helix_data_s
{
    double origin_0;
    double direction_0;
    double origin_1;
    double direction_1;
    double origin_2;
    double direction_2;
    double pitch;
    double radius;
};

/* Table 268 — Type1HelixData */
struct prc_type1_helix_data_s
{
    double unit_z0;
    double unit_u0;
    double unit_z1;
    double unit_u1;
    double unit_z2;
    double unit_u2;
    double reserved_double_0;
    double reserved_double_1;
    double reserved_double_2;
    double reserved_double_3;
    prc_math_fct_1d radius_law;
    prc_math_fct_1d z_law;
    prc_math_fct_1d theta_law;
};

/* Table 266 — PRC_TYPE_CRV_Helix01 */
struct prc_crv_helix01_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    uint8_t type;
    uint8_t orientation;
    prc_vec3 start;
    prc_type0_helix_data type0_helix;
    prc_type1_helix_data type1_helix;
};

/* Table 270 — PRC_TYPE_CRV_Hyperbola */
struct prc_crv_hyperbola_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    double semi_axis;
    double semi_axis_image;
    uint8_t type;
};

/* Table 272 — PRC_TYPE_CRV_Intersection */
struct prc_crv_intersection_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    uint8_t has_transform;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    prc_ptr_surface surface1;
    prc_ptr_surface surface2;
    uint8_t sense_1;
    uint8_t sense_2;
    uint8_t sense_cross;
    uint32_t number_of_crossings;
    prc_crv_crossing_points_intersection *crossing_points;
    prc_vec3 start_limit;
    uint32_t start_limit_type;
    prc_vec3 end_limit;
    uint32_t end_limit_type;
    double chord_error;
    double angle_error;
    uint8_t param_respected;
};

/* Table 273 — CrossingPointsCrvIntersection */
struct prc_crv_crossing_points_intersection_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_vec3 position;
    prc_vec2 uv_surface_1;
    prc_vec2 uv_surface_2;
    prc_vec3 tangent;
    double parameter;
    double scale;
    uint8_t flags;
};

/* Table 276 — PRC_TYPE_CRV_Line */
struct prc_crv_line_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    uint8_t has_transform;
    prc_trans_3d transform;
    prc_parameterization parameterization;
};

/* Table 277 — PRC_TYPE_CRV_Offset */
struct prc_crv_offset_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    prc_ptr_curve base_curve;
    prc_vec3 offset_plane_normal;
    double offset;
};

/* Table 279 — PRC_TYPE_CRV_Parabola */
struct prc_crv_parabola_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    double focal_length;
    uint8_t type;
};

/* Table 282 — PolyLinePoint */
struct prc_polyline_point_s
{
    prc_vec3 point_3d;
    prc_vec2 point_2d;
};

/* Table 281 — PRC_TYPE_CRV_PolyLine */
struct prc_crv_polyline_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    uint8_t has_transform;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    uint32_t number_of_points;
    prc_polyline_point *points;
};

/* Table 284 — PRC_TYPE_CRV_Transform */
struct prc_crv_transform_s
{
    prc_unsigned_int tag;
    prc_content_curve curve_data;
    prc_trans_3d transform;
    prc_parameterization parameterization;
    prc_ptr_curve base_curve;
    prc_math_fct_3d math_transformation;
};

/* Table 180 - PRC_TYPE_TOPO_Context */
struct prc_topo_context_s
{
    prc_unsigned_int tag;
    prc_content_prc_base base;
    uint8_t behavior;
    double grandularity;
    double tolerance;
    uint8_t has_face_thickness;
    double smallest_face_thickness;
    uint8_t has_scale;
    double has_scale_option_undefined_in_spec;
    uint32_t number_of_bodies;
    prc_topo *bodies; /* Not just a body */
};

/* Table 181 */
struct prc_topo_multiple_vertex_s
{
    prc_unsigned_int tag;
    prc_base_topology base;
    prc_unsigned_int number_of_points;
    prc_vec3 *points;
};

/* Table 182 */
struct prc_topo_unique_vertex_s
{
    prc_unsigned_int tag;
    prc_base_topology base;
    prc_vec3 vertex;
    uint8_t has_tolerance;
    double tolerance;
};

/* Table 194 */
struct prc_content_wire_edge_s
{
    prc_base_topology base;
    prc_ptr_curve ptr_curve;
    uint8_t is_trimmed;
    prc_interval trim_interval;
};

/* Table 183 */
struct prc_topo_wire_edge_s
{
    prc_unsigned_int tag;
    prc_content_wire_edge curve;
};

/* Table 184 */
struct prc_topo_edge_s
{
    prc_unsigned_int tag;
    prc_content_wire_edge wire_edge;
    prc_ptr_topology start_vertex;
    prc_ptr_topology end_vertex;
    uint8_t has_tolerance;
    double tolerance;
};

/* Table 195 PRC_TYPE_TOPO_SingleWireBody */
struct prc_topo_single_wire_body_s
{
    prc_unsigned_int tag;
    prc_content_body base;
    prc_ptr_topology wire_body;
};

/* Table 185 */
struct prc_topo_coedge_s
{
    prc_unsigned_int tag;
    prc_base_topology base;
    prc_ptr_topology ptr_topology;
    prc_ptr_curve ptr_curves;
    char coedge_orientation;
    char uv_orientation;
};

/* Table 187 */
struct prc_coedge_in_loop_s
{
    prc_ptr_topology next_coedge;
    prc_unsigned_int neighbor_index;
};

/* Table 186 */
struct prc_topo_loop_s
{
    prc_unsigned_int tag;
    prc_base_topology base;
    char loop_orientation;
    prc_unsigned_int number_of_coedges;
    prc_coedge_in_loop *coedge;
};

/* Table 188 */
struct prc_topo_face_s
{
    prc_unsigned_int tag;
    prc_base_topology base;
    prc_ptr_surface surface_geometry;
    uint8_t is_trimmed;
    prc_domain trimmed_surface;
    uint8_t has_tolerance;
    double tolerance;
    prc_unsigned_int number_of_loops;
    int32_t index_of_outer_loop;
    prc_ptr_topology *loops;
    prc_unsigned_int neighbor_index;
};

/* Table 189 */
struct prc_topo_shell_s
{
    prc_unsigned_int tag;
    prc_base_topology base;
    uint8_t is_closed;
    prc_unsigned_int number_of_faces;
    prc_faces_in_shell *faces;
};

/* Table 190 */
struct prc_faces_in_shell_s
{
    prc_ptr_topology face;
    char orientation;
};

struct prc_topo_connex_s
{
    prc_unsigned_int tag;
    prc_base_topology base;
    prc_unsigned_int number_of_shells;
    prc_ptr_topology *shells;
};

/* Table 117 */
struct prc_ri_brep_model_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    uint8_t exact_geometry;
    prc_unsigned_int index_topological_context;
    prc_unsigned_int index_body;
    uint8_t is_closed;
    prc_user_data user_data;
};

/* Table 118 */
struct prc_ri_curve_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    uint8_t exact_geometry;
    prc_unsigned_int index_topological_context;
    prc_unsigned_int index_body;
    prc_user_data user_data;
};

/* Table 119 */
struct prc_ri_direction_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    prc_ri_curve curve; /* Not in spec */
    uint8_t has_origin;
    prc_vec3 origin;
    prc_vec3 direction;
    prc_user_data user_data;
};

/* Table 120 */
struct prc_ri_plane_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    uint8_t exact_geometry;
    prc_unsigned_int index_topological_context;
    prc_unsigned_int index_body;
    prc_user_data user_data;
};

/* Table 121 */
struct prc_ri_point_set_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    prc_unsigned_int number_of_points;
    prc_vec3 *points;
    prc_user_data user_data;
};

/* Table 122 */
struct prc_ri_poly_brep_model_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    uint8_t is_closed;
    prc_user_data user_data;
};

/* Table 123 */
struct prc_ri_poly_wire_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    prc_user_data user_data;
};

/* Table 124 */
struct prc_ri_set_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    prc_unsigned_int number_of_items;
    prc_ri *rep_items;
    prc_user_data user_data;
};

/* Table 125 */
struct prc_ri_coordinate_system_s
{
    prc_unsigned_int tag;
    prc_representation_item_content item_content;
    prc_misc_transformation transform;
    prc_user_data user_data;
};

/* Nurbs */
typedef struct prc_compressed_weights_s prc_compressed_weights;
typedef struct prc_compressed_knot_s prc_compressed_knot;
typedef struct prc_compressed_knots_s prc_compressed_knots;
typedef struct prc_compressed_knot_vector_s prc_compressed_knot_vector;
typedef struct prc_interior_compressed_control_points_s prc_interior_compressed_control_points;

/* Table 227 CompressedWeights */
struct prc_compressed_weights_s
{
    uint32_t number_bit_weight;
    double *weights;
    double tolerance;
    double *compressed_weights;
};

/* Table 226 CompressedKnot */
struct prc_compressed_knot_s
{
    double knot;
};

/* Table 225 CompressedKnots */
struct prc_compressed_knots_s
{
    uint8_t is_unknown_form;
    uint8_t is_pseudo_uniform;
    uint32_t number_bit_parameter;
    prc_compressed_knot *compressed_knots;
};

/* Table 224 CompressedKnotVector */
struct prc_compressed_knot_vector_s
{
    //uint32_t number_bit_parameter; // Not present.
    uint8_t is_uniform;
    prc_compressed_knots knots;
};

/* Table 223 InteriorCompressedControlPoints */
struct prc_interior_compressed_control_points_s
{
    uint32_t type;
    int32_t Pijz;
    int32_t Pijx;
    int32_t Pijy;
};

typedef struct prc_compressed_control_points_s prc_compressed_control_points;
typedef struct prc_compressed_multiplicities_s prc_compressed_multiplicities;
typedef struct prc_compressed_nurbs_s prc_compressed_nurbs;

/* Table 222 CompressedControlPoints */
struct prc_compressed_control_points_s
{
    prc_vec3 p00;
    prc_vec3 *ccpt_in_v;
    prc_vec3 *ccpt_in_u;
    prc_interior_compressed_control_points *ccpt_interior;
};

/* Table 221 CompressedMultiplicities */
struct prc_compressed_multiplicities_s
{
    uint8_t multiplicity_is_stored;
    uint32_t multiplicity;
};

/* Table 220 CompressedNURBS */
struct prc_compressed_nurbs_s
{
    uint32_t degree_in_u;
    uint32_t degree_in_v;
    uint32_t number_stored_knots_in_u;
    prc_compressed_multiplicities *mult_u;
    uint32_t number_stored_knots_in_v;
    prc_compressed_multiplicities *mult_v;
    uint8_t is_closed_in_u;
    uint8_t is_closed_in_v;
    uint32_t number_bits_for_isomin;
    uint32_t number_bits_for_rest;
    prc_compressed_control_points compressed_control_points;
    prc_compressed_knot_vector knot_vector_u;
    prc_compressed_knot_vector knot_vector_v;
    uint8_t is_rational;
    prc_compressed_weights compressed_weights;
};

/* Table 239 StartEndData */
struct prc_start_end_data_s
{
    uint8_t is_vertex;
    prc_compressed_vertex start_vertex;
    prc_compressed_vertex end_vertex;
    prc_compressed_point start_point;
    prc_compressed_point end_point;
};

/* Table 233 PRC_HCG_Line */
struct prc_hcg_line_s
{
    uint32_t type;
    prc_start_end_data start_end_data;
};

/* Table 235 ParticularCircle */
struct prc_particular_circle_s
{
    uint8_t full_circle;
    prc_start_end_data start_end_data;
    prc_compressed_point center;
    prc_compressed_point normal_plane;
    prc_compressed_point middle_of_arc;
};

/* Table 236 GeneralCircle */
struct prc_general_circle_s
{
    prc_start_end_data start_end_data;
    prc_compressed_point center;
    uint8_t circle_angle; /* Is this really a bool? */
};

/* Table 234 PRC_HCG_Circle */
struct prc_hcg_circle_s
{
    uint32_t type;
    uint8_t is_particular_circle;
    prc_particular_circle particular_circle;
    prc_general_circle general_circle;
};

/* Table 237 PRC_HCG_BsplineHermiteCurve */
struct prc_hcg_bspline_hermite_curve_s
{
    uint32_t type;
    prc_start_end_data start_end_data;
    uint32_t number_bits;
    uint32_t number_points;
    uint32_t point_number_bits;
    prc_vec3 *points;
    prc_vec3 *compressed_points;
    uint32_t tangent_number_bits;
    prc_vec3 *tangents;
    prc_vec3 *compressed_tangents;
};

/* Table 238 PRC_HCG_CompositeCurve */
struct prc_hcg_composite_curve_s
{
    uint32_t type;
    prc_start_end_data start_end_data;
    uint32_t dimension;
    uint8_t is_closed;
    uint32_t number_of_curves;
    prc_ref_or_compressed_curve *curves;
};

/* Table 232 */
struct prc_ref_or_compressed_curve_s
{
    uint8_t curve_is_not_already_stored;
    uint32_t index_compressed_curve;
    prc_compressed_curve *compressed_curve;
};

/* Table 229 */
struct prc_content_compressed_iso_face_s
{
    uint8_t orientation_loop_with_surface;
    prc_ref_or_compressed_curve first_trim_curve;
    prc_ref_or_compressed_curve second_trim_curve;
    prc_ref_or_compressed_curve third_trim_curve_test;
    prc_ref_or_compressed_curve fourth_trim_curve_test;
    uint8_t third_trim_curve_is_not_yet_saved;
    uint32_t third_trim_curve;
    uint8_t fourth_trim_curve_is_not_yet_saved;
    uint32_t fourth_trim_curve;
    prc_compressed_vertex common_third_fourth_vertex;
    prc_compressed_point common_third_fourth_point;
};

/* Table 231 */
struct prc_ana_face_trim_loop_s
{
    uint8_t loop_surface_orientation;
    uint32_t curve_count;
    prc_ref_or_compressed_curve *trim_curves;
    uint8_t curve_is_not_already_stored;
    uint32_t type;
};

/* Table 230 */
struct prc_content_compressed_ana_face_s
{
    uint8_t is_trimmed;
    uint32_t trim_loop_count;
    prc_ana_face_trim_loop *trim_loop;
    prc_compressed_point compressed_point;
};

/* Table 228 */
struct prc_content_compressed_face_s
{
    uint8_t orientation_surface_with_shell;
    uint8_t is_an_iso_face;
    prc_content_compressed_iso_face iso_face;
    prc_content_compressed_ana_face ana_face;
};

struct prc_hcg_iso_plane_s
{
    prc_unsigned_int tag;
    double x, y;
    uint8_t positive_z;
    prc_content_compressed_face face;
};

struct prc_hcg_iso_cylinder_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
};

struct prc_hcg_iso_torus_s
{
    prc_unsigned_int tag;
    uint8_t is_major_radius;
    prc_content_compressed_face face;
};

struct prc_hcg_iso_sphere_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
};

struct prc_hcg_iso_cone_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
};

/* Table 217 IsoNURBSTrimCrv */
struct prc_iso_nurbs_trim_crv_s
{
    uint8_t iso_boundary;
    uint8_t is_a_circle;
    prc_hcg_circle compressed_circle;
    prc_hcg_line compressed_line;
};

/* Table 216 IsoNURBSTrimCurve */
struct prc_iso_nurbs_trim_curve_s
{
    uint8_t is_referenced;
    uint32_t trim_curve_index;
    prc_iso_nurbs_trim_crv trim_curve;
};

/* Table 215 PRC_HCG_IsoNURBS */
struct prc_hcg_iso_nurbs_s
{
    prc_unsigned_int tag;
    uint8_t orientation_surface_with_shell;
    uint8_t orientation_loop_with_surface;
    uint8_t sense_array[3];
    prc_compressed_nurbs surface;
    prc_iso_nurbs_trim_curve curves[4];
    prc_compressed_vertex loop_vertex3;
    prc_compressed_vertex loop_vertex0;
    prc_compressed_vertex loop_vertex1;
    prc_compressed_vertex loop_vertex2;
};

struct prc_hcg_ana_plane_s
{
    prc_unsigned_int tag;
    double x, y;
    uint8_t positive_z;
    prc_content_compressed_face face;
};

struct prc_hcg_ana_cylinder_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
    prc_compressed_point point;
    prc_compressed_point direction;
};

struct prc_hcg_ana_torus_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
    prc_compressed_point center;
    prc_compressed_point x_axis;
    prc_vec3 y_axis;
};

struct prc_hcg_ana_sphere_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
    prc_compressed_point sphere_center;
};

struct prc_hcg_ana_cone_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
    prc_compressed_point axis_point;
    prc_compressed_point apex_point;
};

struct prc_hcg_ana_generic_face_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
    prc_type_surf surface;
};

struct prc_hcg_ana_nurbs_s
{
    prc_unsigned_int tag;
    prc_content_compressed_face face;
    prc_compressed_nurbs compressed_surface;
};

/* Abstract type 8.9.3 PRC_TYPE_TOPO  */
struct prc_topo_s
{
    prc_unsigned_int tag;

    union
    {
        prc_topo_context *topo_context;
        prc_topo_multiple_vertex *topo_multiple_vertex;
        prc_topo_unique_vertex *topo_unique_vertex;
        prc_topo_wire_edge *topo_wire_edge;
        prc_topo_edge *topo_edge;
        prc_topo_coedge *topo_coedge;
        prc_topo_loop *topo_loop;
        prc_topo_face *topo_face;
        prc_topo_shell *topo_shell;
        prc_topo_connex *topo_connex;
        prc_topo *topo_body;
        prc_topo_single_wire_body *topo_single_wire_body;
        prc_topo_brep_data *topo_brep_data;
        prc_topo_single_wire_compress *topo_single_wire_compress;
        prc_topo_brep_data_compress *topo_brep_data_compress;
    };
};

/* Abstract type 8.9.15 prc_type_topo_body */
struct prc_topo_body_s
{
    prc_topological_entity_t topo_body_type;
    prc_content_body base;

    union
    {
        prc_topo_single_wire_body *topo_single_wire_body;
        prc_topo_brep_data *topo_brep_data;
        prc_topo_single_wire_compress *topo_single_wire_compress;
        prc_topo_brep_data_compress *topo_brep_data_compress;
    };
};

/* Abstract 8.6.2 prc_ri */
struct prc_ri_s
{
    prc_representation_item_t representation_type;
    prc_representation_item_content item_content;

    union
    {
        prc_ri *ri_rep_item;
        prc_ri_brep_model *ri_brep_model;
        prc_ri_curve *ri_curve;
        prc_ri_direction *ri_direction;
        prc_ri_plane *ri_plane;
        prc_ri_point_set *ri_point_set;
        prc_ri_poly_brep_model *ri_poly_brep_model;
        prc_ri_poly_wire *ri_poly_wire;
        prc_ri_set *ri_set;
        prc_ri_coordinate_system *ri_coordinate_system;
    };
};

/* Abstract 8.7.7 */
struct prc_annotation_entity_s
{
    prc_unsigned_int tag;
    union
    {
        prc_type_mkp_annotation_item item;
        prc_type_mkp_annotation_set set;
        prc_type_mkp_annotation_reference ref;
    };
};

/* Section 8.9.21.5 CompressedFace */
struct prc_compressed_face_s
{
    /* A large abstract type. */
    prc_unsigned_int tag;
    union
    {
        prc_hcg_iso_plane hcg_iso_plane;
        prc_hcg_iso_cylinder hcg_iso_cylinder;
        prc_hcg_iso_torus hcg_iso_torus;
        prc_hcg_iso_sphere hcg_iso_sphere;
        prc_hcg_iso_cone hcg_iso_cone;
        prc_hcg_iso_nurbs hcg_iso_nurbs;
        prc_hcg_ana_plane hcg_ana_plane;
        prc_hcg_ana_cylinder hcg_ana_cylinder;
        prc_hcg_ana_torus hcg_ana_torus;
        prc_hcg_ana_sphere hcg_ana_sphere;
        prc_hcg_ana_cone hcg_ana_cone;
        prc_hcg_ana_generic_face hcg_ana_generic_face;
        prc_hcg_ana_nurbs hcg_ana_nurbs;
    };
};

struct prc_compressed_shell_s
{
    uint8_t single_face;
    prc_unsigned_int number_of_faces;  /* Odd encode */
    prc_compressed_face *faces;
    uint8_t *is_iso_face;
};

/* Table 201 CompressedConnex */
struct prc_compressed_connex_s
{
    prc_unsigned_int number_of_shells;
    prc_compressed_shell *shells;
};

/* Table 200 MultipleCompressedConnex */
struct prc_multi_compressed_connex_s
{
    prc_unsigned_int number_of_connex;
    prc_compressed_connex *connex;
    uint32_t number_of_faces;
    /* This is not in the spec, but it is needed to know how many faces are stored across the connexes. */
};

/* Section 8.9.21.9 */
struct prc_compressed_curve_s
{
    PRC_Compressed_curve_t curve_type;
    union
    {
        prc_hcg_line hcg_line;
        prc_hcg_circle hcg_circle;
        prc_hcg_bspline_hermite_curve hcg_bspline_hermite_curve;
        prc_hcg_composite_curve hcg_composite_curve;
    };
};

/* Table 197 PRC_TYPE_TOPO_SingleWireBodyCompress */
struct prc_topo_single_wire_compress_s
{
    prc_unsigned_int tag;
    prc_content_body base;
    double curve_tolerance;
    prc_ptr_topology *connex;
    prc_compressed_curve compressed_curve;
};

/* Table 198 PRC_TYPE_TOPO_BrepDataCompress */
struct prc_topo_brep_data_compress_s
{
    prc_unsigned_int tag;
    prc_content_body base;
    double brep_data_compressed_tolerance;
    prc_unsigned_int number_of_bits_to_store_ref;
    prc_unsigned_int number_of_vertex_refs;
    prc_unsigned_int number_of_edge_refs;
    uint8_t single_connex_test;
    prc_compressed_shell single_connex;
    prc_multi_compressed_connex multi_connex;
    prc_base_topology *base_topology;
    uint32_t number_of_faces;
};

/* A way to organize across files, tessellations, parts */
typedef struct tess_style_file_part_s
{
    uint32_t part_reserve_index; /* The index of the part in the reserve. */
    uint32_t tessellation_index; /* The index of the tessellation for the part. */
    uint32_t tess_file_index;    /* Which file index is this tessellation defined */
    void *tessellation;          /* The tessellation for the part, from the viewer application */
    void *wire_tessellation;     /* 3D tessellation may have wire data.  Store that in a different tess */
    uint32_t style_biased_index; /* The index of the style based for the part. */
    uint8_t has_inherited_style; /* Whether the part has an inherited style or not. */
    uint8_t has_inherited_color; /* Whether the part has an inherited style or not. */
    uint8_t has_inherited_trans; /* Whether the part has an inherited style or not. */
    uint32_t inherited_color_index; /* The index of the inheritedstyle color for the part. */
    uint32_t inherited_trans_index; /* The index of the inheritedstyle transparency for the part. */
    uint32_t inherited_color_file_index; /* Which file index is this inherited color defined */
    uint32_t inherited_trans_file_index; /* Which file index is this inherited transparency defined */
    uint8_t has_entity_ref;
    prc_api_entity_ref entity_ref; /* The entity reference for the part, if it exists. */
    void *style_leaf; /* The style leaf for the part, from the viewer application. */
} tess_style_file_part;

typedef struct tess_style_file_markup_s
{
    uint32_t markup_reserve_index; /* The index of the markup in the reserve. */
    uint32_t tessellation_index; /* The index of the tessellation for the markup. */
    uint32_t tess_file_index;    /* Which file index is this tessellation defined */
    void *tessellation;          /* The tessellation for the markup, from the viewer application */
    uint32_t style_biased_index; /* The index of the style based for the markup. */
} tess_style_file_markup;

struct prc_data_s
{
    prc_header *header;
    uint32_t file_structure_count;
    prc_filestructure *file_struct;
    uint32_t view_count;
    prc_pdf_view_array *views;
    uint32_t unique_part_count;
    tess_style_file_part *part_details;
    uint32_t unique_markup_count;
    tess_style_file_markup *markup_details;
};

struct prc_triangle_s
{
    size_t indices[3];
};


/* Data types returned by schema methods */
struct prc_schema_read_s
{
    int in_for_loop;
    uint32_t loop_value;
    prc_schema_tokens_t data_type;
    union
    {
        unsigned char bool_val;
        double double_val;
        unsigned char char_val;
        uint32_t uint32_val;
        int32_t int32_val;
        prc_string string_val;
        prc_vec2 vec2_val;
        prc_vec3 vec3_val;
        prc_interval interval_val;
        prc_domain domain_val;
        prc_bounding_box bbox_val;
        prc_unsigned_int parent_type;
    };
};

/* A structure to maintain compressed brep data as vertices and curve
   should be stored as they are decoded. They are then potentially
   referenced via an index */
struct prc_nano_brep_compressed_data_s
{
    prc_unsigned_int number_of_vertex_refs;
    uint32_t vertices_capacity;
    prc_compressed_vertex *vertices;
    prc_unsigned_int number_of_edge_refs;
    uint32_t curves_capacity;
    prc_compressed_curve *curves;
    uint32_t number_bits_for_encoding;
    uint32_t current_vertex_index; /* Index of next vertex to be added */
    uint32_t current_curve_index; /* Index of next curve to be added */
    uint8_t compressed_iso_spline; /* Ugly about this one. True if curve is being used as trim boundry for PRC_HCG_IsoNURBS */
    uint8_t is_iso_type; /* True if the face being decoded is an iso type */
    uint8_t curve_trimming_face; /* Another ugly one */
    double tolerance;
    uint8_t all_loops_are_vertex_loops;
    uint8_t surface_type_is_prc_hcg_ana_torus;
    uint8_t is_a_compressed_face;
};

prc_data* prc_open_contents(prc_context *ctx, const char *infile);
void prc_release_data(prc_context *ctx, prc_data *data);

#endif
