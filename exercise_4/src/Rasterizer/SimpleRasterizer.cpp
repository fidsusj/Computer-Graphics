#include <algorithm>

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <gtc/matrix_inverse.hpp>

#define RAYTRACER_USE_FOREACH
#include <Raytracer/Raytracer.h>

#include <Rasterizer/SimpleRasterizer.h>

using namespace std;
using namespace glm;
using namespace Rasterizer;
using namespace Raytracer;
using namespace Raytracer::Objects;
using namespace Raytracer::Scenes;

SimpleRasterizer::SimpleRasterizer()
{
  ambientLight = vec3(0.01f);
}

bool SimpleRasterizer::CompareTriangle(const Triangle &t1, const Triangle &t2)
{
  // These aren't actually the mean values, but since both are off by a constant factor (3),
  // this inequation is equivalent.
  return (t1.position[0].z + t1.position[1].z + t1.position[2].z >
    t2.position[0].z + t2.position[1].z + t2.position[2].z);
}


void SimpleRasterizer::DrawSpan(int x1, int x2, int y, float z1, float z2, vec3 &color1, vec3 &color2)
{
    // TODO Aufgabe 2: Ersetzen des Zeichnens der Eckpunkte 
    // durch Dreiecksrasterisierer, Gouraud Shading, [z-Buffering]
    
    // Determine ordering of x1 and x2
    int xMin = x1 <= x2 ? x1 : x2;
    int xMax = x1 > x2 ? x1 : x2;

    // limit xMin and xMax to the size of the image
    xMin = clamp(xMin, 0, image->GetWidth());
    xMax = clamp(xMax, 0, image->GetWidth());

    const double zDelta = ((double) z2 - z1) / ((double) xMax - xMin);
    const int xDelta = xMax - xMin;
    const int yOffset = y * image->GetWidth();
    double z = z1;

    // draw span
    int t = 0;
    for (int x = xMin; x < xMax; ++x) {
        if (z < zBuffer[x + yOffset]) {
            zBuffer[x + yOffset] = z;
            auto color = color1 * (1 - (t / (float)xDelta)) + color2 * (t / (float)xDelta);
            image->SetPixel(x, y, color);
        }

        z += zDelta;
        t++;
    }
}

void SimpleRasterizer::DrawTriangle(const Triangle& t)
{
    // for (int i = 0; i < 3; ++i)
    // {
    //     int x = (int)t.position[i].x;
    //     int y = (int)t.position[i].y;
    //     if ((x > 0) && (x < image->GetWidth()) && (y > 0) && (y < image->GetHeight()))
    //     {
    //         image->SetPixel(x, y, t.color[i]);
    //     }
    // }

    // TODO Aufgabe 2: Ersetzen des Zeichnens der Eckpunkte 
    // durch Dreiecksrasterisierer, Gouraud Shading, [z-Buffering]
    float x_t = numeric_limits<float>::max();
    float y_t = numeric_limits<float>::max();
    int y_b = 0;

    int p1_index = 0;
    int p2_index = 0;
    int p3_index = 0;

    // Search for P2 index
    for (int i = 0; i < 3; i++)
    {
        const float x = t.position[i].x;
        const float y = t.position[i].y;

        if (y < y_t)
        {
            p2_index = i;
            x_t = x;
            y_t = y;
        }

        // Determine y_b on the run
        if (y > y_b)
        {
            y_b = int(round(y));
        }
    }


    // Search for P1 index
    float min_x = numeric_limits<float>::max();
    for (int i = 0; i < 3; i++)
    {
        if (i == p2_index)
            continue;
      
        const float x = t.position[i].x;

        if (x < min_x)
        {
            p1_index = i;
            min_x = x;
        }
    }

    // Search for P3 index
    for (int i = 0; i < 3; i++)
    {
        if (i == p1_index || i == p2_index)
            continue;

        p3_index = i;
    }

    // Sort points of triangle so that P2 is topmost point, P1 leftmost point (ignoring P2), P3 remaining point
    vec3 points[3] = { t.position[p1_index], t.position[p2_index], t.position[p3_index] };

    int y = int(round(y_t));
  
    float x_l = x_t;
    int cur_l = 1;
    int next_l = cur_l - 1;

    float x_r = x_t;
    int cur_r = 1;
    int next_r = cur_r + 1;

    // We do not do a perspectively correct linear interpolation of the color for now
    // Therefore we hard code the color used for every pixel and always pass 0 as z value for DrawSpan.
    vec3 color = t.color[0];

    do
    {
        if (next_l < 0)
            next_l = 2;

        if (next_r > 2)
            next_r = 0;

        DrawSpan(int(round(x_l)), int(round(x_r)), y, 0, 0, color, color);
    
        y++;

        x_l += (points[next_l].x - points[cur_l].x) / (points[next_l].y - points[cur_l].y);
        x_r += (points[next_r].x - points[cur_r].x) / (points[next_r].y - points[cur_r].y);

        int y_next_l = int(round(points[next_l].y));
        int y_next_r = int(round(points[next_r].y));
    
        if (y >= y_next_l)
        {
            cur_l = next_l;
            next_l--;
        }

        if (y >= y_next_r)
        {
            cur_r = next_r;
            next_r++;
        }
    } while (y < y_b);
}


vec3 SimpleRasterizer::LightVertex(vec4 position, vec3 normal, vec3 color)
{
  vec3 result = color * ambientLight;

  foreach (const Light *, light, lights)
  {
    vec3 intensity = vec3(1.0f);
    if ((*light)->IsInstanceOf(SceneObjectType_PointLight))
      intensity = ((PointLight *)*light)->GetIntensity();

    vec3 distance = (*light)->GetPosition() - vec3(position);
    float attenuation = 1.0f / (0.001f + dot(distance, distance));
    vec3 direction = normalize(distance);

    float lambert = glm::max(0.0f, dot(normal, direction));

    if (lambert > 0)
      result += color * lambert * attenuation * intensity;
  }

  return result;
}


void SimpleRasterizer::SortTriangles(vector<Triangle> &triangles)
{
  sort(triangles.begin(), triangles.end(), CompareTriangle);
}


void SimpleRasterizer::TransformAndLightTriangle(Triangle &t, 
                                                 const mat4 &modelTransform,
                                                 const mat4 &modelTransformNormals)
{
  for (int i = 0; i < 3; i++)
  {
    vec4 worldPos = modelTransform * vec4(t.position[i], 1.0f);
    vec3 normal = normalize(vec3(modelTransformNormals * vec4(t.normal[i], 0.0f)));

    t.color[i] = LightVertex(worldPos, normal, t.color[i]);

    vec4 pos = viewProjectionTransform * worldPos;

    // Transform to screen coordinates
    t.position[i].x = (pos.x / pos.w * 0.5f + 0.5f) * image->GetWidth();
    t.position[i].y = (pos.y / pos.w * -0.5f + 0.5f) * image->GetHeight();
    t.position[i].z = pos.z / pos.w;
  }
}


void SimpleRasterizer::RenderMesh(const Mesh *mesh)
{
  if (mesh == NULL)
    return;

  // Compute the transformation from model to world space and its transposed inverse (for the
  // normals).
  glm::mat4 modelTransform = mesh->GetGlobalTransformation();
  glm::mat4 transposedInverseMT = inverseTranspose(modelTransform);

  // Transform and light all triangles in the mesh.
  const vector<Triangle> &meshTriangles = mesh->GetTriangles();
  vector<Triangle> renderTriangles;

  foreach_c(Triangle, triangle, meshTriangles)
  {
    Triangle t = *triangle;
    TransformAndLightTriangle(t, modelTransform, transposedInverseMT);
    renderTriangles.push_back(t);
  }

  //TODO Aufgabe 2: Painter's Algorithm
  // Draw the triangles.
  foreach(Triangle, triangle, renderTriangles)
    DrawTriangle(*triangle);
}

void SimpleRasterizer::ScanObject(const Raytracer::Scenes::SceneObject *object)
{
  if (object == NULL)
    return;

  if (object->IsInstanceOf(SceneObjectType_Light))
    lights.push_back((const Light *)object);
  if (object->IsInstanceOf(SceneObjectType_Mesh))
    meshes.push_back((const Mesh *)object);

  foreach_c(SceneObject *, child, object->GetChildren())
    ScanObject(*child);
}

bool SimpleRasterizer::Render(Image &image, const Scene &scene)
{
  image.Clear(vec3(0));

  Camera *camera = scene.GetActiveCamera();
  if (camera == NULL)
    return false;

  zBuffer = new float[image.GetWidth() * image.GetHeight()];
  for (int i = 0; i < image.GetWidth() * image.GetHeight(); i++)
    zBuffer[i] = 1.0f;

  // Build lists of all lights and meshes in the scene.
  lights.clear();
  meshes.clear();
  ScanObject(&scene);

  mat4 projectionTransform = perspective(camera->GetFov(),camera->GetAspect(), 
    camera->GetNearClip(), camera->GetFarClip());
  mat4 viewTransform = lookAt(camera->GetEye(), camera->GetLookAt(), camera->GetUp());
  viewProjectionTransform = projectionTransform * viewTransform;

  // Render all meshes we found.
  this->image = &image;
  foreach(const Mesh *, mesh, meshes)
    RenderMesh(*mesh);

  delete[] zBuffer;

  return true;
}
