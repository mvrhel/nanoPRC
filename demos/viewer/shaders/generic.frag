#version 330 core

layout (location = 0) out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;
in vec3 Color;
in vec4 FragPosLightSpace;
in vec3 VDiffuseColor;
in vec3 VMaterialTint;
in vec3 VSpecularColor;
in vec3 VEmissiveColor;
in float VShininess;
in float VAlpha;

struct DirLight
{
    vec3 direction;
    vec3 color;
    float intensity;
};

struct PointLight
{
    vec3 position;
    vec3 color;
    float intensity;
};

struct Material
{
    vec3 diffuse;
    vec3 specular;
    float shininess;
    float alpha;
    vec3 tint;

    sampler2D diffuseMap;
    bool hasDiffuseMap;

    sampler2D specularMap;
    bool hasSpecularMap;
};

uniform vec3 uViewPos;

uniform DirLight uDirLight;
uniform PointLight uPointLight;

uniform vec3 uSkyColorHorizon;
uniform vec3 uSkyColorZenith;

uniform Material uMaterial;
uniform bool uIsLine;
uniform bool uVertexMaterial;

uniform sampler2D uShadowMap;
uniform bool uEnableShadows;

uniform bool uWireframe;
uniform bool uFullbright;
uniform float uAmbientWeight;
uniform float uDiffuseWeight;
uniform int uDebugMode;

const float kShadowBias = 0.002;
const float kNormalEpsilon = 1e-5;
const float kDiffuseStrength = 1.15;
const float kSpecularStrength = 0.65;
const float kShadedSaturation = 1.03;
const float kShadedGamma = 0.99;

float getSpecularPower(float shininess)
{
    /* Some imported materials provide shininess in [0,1] rather than Phong exponent space. */
    if (shininess <= 1.0)
        return mix(4.0, 128.0, clamp(shininess, 0.0, 1.0));

    return max(shininess, 1.0);
}

vec3 getShadingNormal()
{
    vec3 interpNormal = gl_FrontFacing ? Normal : -Normal;
    float normalLen = length(interpNormal);

    if (normalLen > kNormalEpsilon)
    {
        /* Interpolated normals are not unit length in fragment stage. */
        return interpNormal / normalLen;
    }

    /* Fallback to geometric normal when imported normals are missing/degenerate. */
    vec3 geomNormal = normalize(cross(dFdx(FragPos), dFdy(FragPos)));
    return gl_FrontFacing ? geomNormal : -geomNormal;
}

vec3 boostShadedColor(vec3 color)
{
    color = max(color, vec3(0.0));

    float luma = dot(color, vec3(0.2126, 0.7152, 0.0722));
    color = mix(vec3(luma), color, kShadedSaturation);

    return clamp(pow(color, vec3(kShadedGamma)), 0.0, 1.0);
}

/* Calculate visibility of fragment from light source */
float calcShadowFactor(vec4 fragPosLightSpace, vec3 normal, vec3 lightDir)
{
    if (!uEnableShadows)
        return 1.0;

    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;

    /* Check if fragment is outside of light frustum */
    if (projCoords.z > 1.0 || projCoords.z < 0.0 || projCoords.x > 1.0 || projCoords.x < 0.0 || projCoords.y > 1.0 || projCoords.y < 0.0)
        return 1.0;
    
    float closestDepth = texture(uShadowMap, projCoords.xy).r;
    float currentDepth = projCoords.z;

    float bias = max(0.03 * (1.0 - dot(normal, lightDir)), kShadowBias);
    return currentDepth - bias > closestDepth ? 0.0 : 1.0;
}

/* Calculate diffuse lighting from a directional light (sun) */
vec3 calcDirDiffuse(DirLight light, vec3 normal, vec3 diffuseContrib)
{
    if (light.intensity <= 0.0)
        return vec3(0.0);

    vec3 L = normalize(-light.direction);

    float diff = max(dot(normal, L), 0.0);
    if (diff <= 0.0)
        return vec3(0.0);

    float shadowFactor = calcShadowFactor(FragPosLightSpace, normal, L);
    if (shadowFactor <= 0.0)
        return vec3(0.0);

    return diff * diffuseContrib * light.color * light.intensity * shadowFactor * kDiffuseStrength * uDiffuseWeight;
}

/* Calculate specular lighting from a directional light (sun) */
vec3 calcDirSpecular(DirLight light, vec3 normal, vec3 specularContrib,
    float shininess)
{
    if (light.intensity <= 0.0)
        return vec3(0.0);

    vec3 L = normalize(-light.direction);
    vec3 V = normalize(uViewPos - FragPos);
    float specularPower = getSpecularPower(shininess);

    float diff = max(dot(normal, L), 0.0);
    if (diff <= 0.0)
        return vec3(0.0);

    float shadowFactor = calcShadowFactor(FragPosLightSpace, normal, L);
    if (shadowFactor <= 0.0)
        return vec3(0.0);

    vec3 specular = vec3(0.0);
    vec3 R = 2 * dot(normal, L) * normal - L;
    specular = specularContrib * pow(max(dot(R, V), 0.0), specularPower);

    return specular * light.color * light.intensity * shadowFactor * kSpecularStrength;
}

/* Calculate diffuse lighting from a point light */
vec3 calcPointDiffuse(PointLight light, vec3 normal, vec3 fragPos,
    vec3 diffuseContrib)
{
    if (light.intensity <= 0.0)
        return vec3(0.0);

    vec3 L = normalize(light.position - fragPos);

    float diff = max(dot(normal, L), 0.0);
    vec3 diffuse = diff * diffuseContrib;

    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / distance;

    return diffuse * light.color * attenuation * light.intensity * kDiffuseStrength * uDiffuseWeight;
}

/* Calculate specular lighting from a point light */
vec3 calcPointSpecular(PointLight light, vec3 normal, vec3 fragPos,
    vec3 specularContrib, float shininess)
{
    if (light.intensity <= 0.0)
        return vec3(0.0);

    vec3 L = normalize(light.position - fragPos);
    vec3 V = normalize(uViewPos - fragPos);
    vec3 H = normalize(L + V);
    float specularPower = getSpecularPower(shininess);

    float diff = max(dot(normal, L), 0.0);
    if (diff <= 0.0)
        return vec3(0.0);

    float spec = pow(max(dot(normal, H), 0.0), specularPower);
    vec3 specular = spec * specularContrib;

    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / distance;

    return specular * light.color * attenuation * light.intensity * kSpecularStrength;
}

void main()
{
    if (uDebugMode == 4)
    {
        /* Branch visualization:
           yellow=wireframe, cyan=line/markup, magenta=fullbright,
           green=vertex-material, blue=uniform-material */
        if (uWireframe)
        {
            FragColor = vec4(1.0, 1.0, 0.0, 1.0);
            return;
        }
        if (uIsLine)
        {
            FragColor = vec4(0.0, 1.0, 1.0, 1.0);
            return;
        }
        if (uFullbright)
        {
            FragColor = vec4(1.0, 0.0, 1.0, 1.0);
            return;
        }
        if (uVertexMaterial)
        {
            FragColor = vec4(0.0, 1.0, 0.0, 1.0);
            return;
        }

        FragColor = vec4(0.0, 0.0, 1.0, 1.0);
        return;
    }

    // DEBUG: Test if back-facing fragments are even executing
   // FragColor = gl_FrontFacing ? vec4(1.0, 0.0, 0.0, 1.0) : vec4(0.0, 0.0, 1.0, 1.0);
   // return;

    if (uWireframe)
    {
        FragColor = vec4(1.0, 0.0, 0.0, 1.0);
        return;
    }
	
	if (uIsLine)
	{
        // For markups (lines and text) we want an unlit path
        if (uMaterial.hasDiffuseMap)
        {
            // TEXT RENDERING: Font atlas glyph coverage in ALPHA channel
            vec4 tex = texture(uMaterial.diffuseMap, TexCoords);
            
            // Font atlas: RGB=white (ignored), Alpha=glyph coverage (0-1)
            float glyphCoverage = tex.a;
            
            // Vertex color contains the desired text color
            vec3 textColor = Color * uMaterial.tint;
            
            // Use glyph coverage as the alpha for the text color
            FragColor = vec4(textColor, glyphCoverage * uMaterial.alpha);
           // FragColor = vec4(textColor, glyphCoverage);
        }
        else
        {
            // LINE RENDERING: Plain color (no texture)
            //FragColor = vec4(Color * uMaterial.tint, uMaterial.alpha);
            FragColor = vec4(Color, 1.0);
        }
		return;
	}

    {
        vec3 debugNormal = getShadingNormal();
        float rawNormalLen = length(Normal);
        vec3 rawNormal = rawNormalLen > kNormalEpsilon ? Normal / rawNormalLen
                                                       : vec3(0.0, 0.0, 1.0);

        vec3 dpx = dFdx(FragPos);
        vec3 dpy = dFdy(FragPos);
        vec3 geomCross = cross(dpx, dpy);
        float geomLen = length(geomCross);
        vec3 geomNormal = geomLen > kNormalEpsilon ? geomCross / geomLen
                                                   : vec3(0.0, 0.0, 1.0);

        if (uDebugMode == 1)
        {
            /* RGB view of final shading normal in world space. */
            FragColor = vec4(debugNormal * 0.5 + 0.5, 1.0);
            return;
        }
        if (uDebugMode == 2)
        {
            /* Length of incoming interpolated normal, white should be ~1. */
            float l = clamp(rawNormalLen, 0.0, 1.0);
            FragColor = vec4(l, l, l, 1.0);
            return;
        }
        if (uDebugMode == 3)
        {
            /* Red marks likely missing/degenerate normals. */
            FragColor = rawNormalLen > kNormalEpsilon ?
                vec4(0.0, 1.0, 0.0, 1.0) : vec4(1.0, 0.0, 0.0, 1.0);
            return;
        }
        if (uDebugMode == 5)
        {
            /* Winding/front-face visualization: red=front, blue=back. */
            FragColor = gl_FrontFacing ? vec4(1.0, 0.0, 0.0, 1.0)
                                       : vec4(0.0, 0.0, 1.0, 1.0);
            return;
        }
        if (uDebugMode == 6)
        {
            /* Raw interpolated normal only (no gl_FrontFacing flip/fallback). */
            FragColor = vec4(rawNormal * 0.5 + 0.5, 1.0);
            return;
        }
        if (uDebugMode == 7)
        {
            /* Imported-vs-geometric normal agreement: red=opposed, green=aligned. */
            float d = dot(rawNormal, geomNormal);
            float agree = clamp(d * 0.5 + 0.5, 0.0, 1.0);
            FragColor = vec4(1.0 - agree, agree, 0.0, 1.0);
            return;
        }
    }

    if (uFullbright)
    {
        vec3 color;
        float alpha;

        if (uVertexMaterial)
        {
            color = VDiffuseColor;
            alpha = clamp(VAlpha * uMaterial.alpha, 0.0, 1.0);
        }
        else
        {
            vec4 texel = uMaterial.hasDiffuseMap ? texture(uMaterial.diffuseMap, TexCoords)
                                                 : vec4(uMaterial.diffuse, 1.0);
            color = texel.rgb;
            alpha = texel.a * uMaterial.alpha;
        }

        FragColor = vec4(clamp(color, 0.0, 1.0), clamp(alpha, 0.0, 1.0));
		//FragColor = vec4(TexCoords, 0.0, 1.0); /* For displaying texture coordinates */
        return;
    }

    if (uVertexMaterial)
    {
        vec3 normal = getShadingNormal();

        /* Compute ambient color */
        float cosTheta = max(dot(normal, vec3(0.0, 1.0, 0.0)), 0.0);
        vec3 ambient = mix(uSkyColorHorizon, uSkyColorZenith, cosTheta);

        vec3 diffuseContrib = VDiffuseColor;
        vec3 specularContrib = VSpecularColor;
        float shininess = VShininess;
        
        /* Compute lighting */
        vec3 ambientTerm = ambient * VMaterialTint * uAmbientWeight;
        vec3 diffuseTerm = calcDirDiffuse(uDirLight, normal, diffuseContrib);
        diffuseTerm += calcPointDiffuse(uPointLight, normal, FragPos,
            diffuseContrib);
        vec3 specularTerm = calcDirSpecular(uDirLight, normal, specularContrib,
            shininess);
        specularTerm += calcPointSpecular(uPointLight, normal, FragPos,
            specularContrib, shininess);

          /* Final color: for vertex-material shading, use material channels directly.
              Multiplying by vertex Color can desaturate intended PRC diffuse colors. */
          vec3 color = ambientTerm + diffuseTerm + specularTerm;
        // FragColor = vec4(normal * 0.5 + 0.5, 1.0);  /* Debug of normal vectors */

        FragColor = vec4(clamp(color, 0.0, 1.0),
            clamp(VAlpha * uMaterial.alpha, 0.0, 1.0));

                 
        return;
    }

    vec3 normal = getShadingNormal();

    /* Compute ambient color */
    float cosTheta = max(dot(normal, vec3(0.0, 1.0, 0.0)), 0.0);
    vec3 ambient = mix(uSkyColorHorizon, uSkyColorZenith, cosTheta) * 0.42; // Keep some depth without washing out color

    /* Sample textures */
    vec3 diffuseContrib = uMaterial.hasDiffuseMap ? texture(uMaterial.diffuseMap, TexCoords).rgb : uMaterial.diffuse;
    vec3 specularContrib = uMaterial.hasSpecularMap ? texture(uMaterial.specularMap, TexCoords).rgb : uMaterial.specular;
    float alphaDiffuse = uMaterial.hasDiffuseMap ? texture(uMaterial.diffuseMap, TexCoords).a : 1.0;
    float alphaSpecular = uMaterial.hasSpecularMap ? texture(uMaterial.specularMap, TexCoords).a : 1.0;
    float alpha = min(alphaDiffuse, alphaSpecular);
    
    /* Compute lighting */
    vec3 ambientTerm = ambient * Color * diffuseContrib * uAmbientWeight;
    vec3 diffuseTerm = calcDirDiffuse(uDirLight, normal, diffuseContrib);
    diffuseTerm += calcPointDiffuse(uPointLight, normal, FragPos,
        diffuseContrib);
    vec3 specularTerm = calcDirSpecular(uDirLight, normal, specularContrib,
        uMaterial.shininess);
    specularTerm += calcPointSpecular(uPointLight, normal, FragPos,
        specularContrib, uMaterial.shininess);

    /* Final color.  Tint is ambient */
    //vec3 color = 0.1 * uMaterial.tint + ambientTerm + diffuseTerm * Color + specularTerm;
    vec3 color = ambientTerm + diffuseTerm * Color + specularTerm;

	// TEMPORARY DEBUG - replace with this line:
   // color = gl_FrontFacing ? vec3(1.0, 0.0, 0.0) : vec3(0.0, 0.0, 1.0);
	// DEBUG: Visualize alpha values
  //  color = vec3(uMaterial.alpha, alphaDiffuse, alpha);
  

    // FragColor = vec4(normal * 0.5 + 0.5, 1.0);  /* Debug of normal vectors */

    //FragColor = vec4(alphaDiffuse, alphaSpecular, 0, 1.0);
   // FragColor = vec4(color, uMaterial.alpha);

       // DEBUG: Visualize alpha values - REMOVE AFTER TESTING
    //color = vec3(alpha, uMaterial.alpha, alpha * uMaterial.alpha);

     //  FragColor = vec4(boostShadedColor(color), alpha * uMaterial.alpha);
     FragColor = vec4(color, 1.0);

    return;
}
