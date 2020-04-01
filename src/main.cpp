//------------------------------------------------------------------------------
// A simple example showing how to use the triangle geometry
//------------------------------------------------------------------------------
#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>

#include "io.h"
#include "turntable_controls.h"


#define null nullptr

using namespace glm;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;
using std::vector;
using givr::vec3f;
using givr::mat4f;

struct Mass;
struct Spring;
struct MassSpringSystem;
struct Plane;
struct Controler;
MassSpringSystem CreateMassSpringLine(vec3f origin, vec3f extent, int masses, float massWeight);
MassSpringSystem CreateMassSpringRect(vec3f origin, float widthX, float heightY, float depthZ, int massesPerAxis, float massWeight);
MassSpringSystem CreateMassSpringCloth(vec3f origin, float widthX, float heightY, int xMasses, int yMasses, float massWeight);
void CalculateCollisionForce(Mass* mass, Plane ground, float elasticity, float sphereRadius);
bool FloatEquals(float lhs, float rhs);
bool VecIsZero(vec3f vec);
uint Flatten(int i, int j, int k, int indicesPerAxis);

struct Plane
{
   vec3f Position;
   vec3f Normal;
   Plane(vec3f position, vec3f normal): Position(position), Normal(normalize(normal)) {}

};

struct Mass
{
   float Weight;
   vec3f Position;
   vec3f ForceAccumulation;
   vec3f Velocity;
   bool IsFixed;
   Mass(vec3f position, float weight, bool isFixed = false)
   {
      Position = position;
      Weight = weight;
      ForceAccumulation = vec3f(0, 0, 0);
      Velocity = vec3f(0, 0, 0);
      IsFixed = isFixed;
   }
};

struct Spring
{
   Mass* Mass1;
   Mass* Mass2;
   float RestLength;
   float K;
   float Dampening;
   Spring(Mass* m1, Mass* m2, float k = 1, float damp = 0.5)
   {
      Mass1 = m1;
      Mass2 = m2;
      RestLength = length(m1->Position - m2->Position);
      K = k;
      Dampening = damp;
   }

};

struct MassSpringSystem
{
   vector<Mass*> Masses;
   vector<Spring> Springs;

   // An override for individual spring constants to quickly adjust an entire system.
   float SpringKValue = 1.f;
   float SpringDampeningValue = 0.5f;
   float CollisionElasticity = 0.8f;

   float PointScaleFactor = 0.5f;

   void AddMass(Mass* m)
   {
      Masses.push_back(m);
   }

   void AddSpring(Spring s)
   {
      Springs.push_back(s);
   }

   void AddSpring(int i, int j, float k = 5, float damp = 0.1)
   {
      Springs.push_back(Spring(Masses[i], Masses[j], k, damp));
   }

   void DoTimestep(Plane* ground = null)
   {
      float deltaTime = 0.001f;
      for (auto & mass : Masses)
      {
         mass->ForceAccumulation = vec3f(0);
      }

      for (auto& spring : Springs)
      {
         const vec3f& i = spring.Mass1->Position;
         const vec3f& j = spring.Mass2->Position;

         vec3f jToi = i - j;
         const float len = length(jToi);
         vec3f springForce = -SpringKValue * (len - spring.RestLength) * (jToi / len);

         vec3f dampForce = { 0,0,0 };
         if (!VecIsZero(springForce))
         {
            dampForce = -spring.Dampening * (dot(spring.Mass1->Velocity - spring.Mass2->Velocity, springForce) / dot(springForce, springForce)) * springForce;
         }
         spring.Mass1->ForceAccumulation += springForce + dampForce;
         spring.Mass2->ForceAccumulation += -springForce - dampForce;
      }

      for (auto& mass : Masses)
      {
         vec3f gravityForce = { 0, -9.81 * mass->Weight, 0 };
         mass->ForceAccumulation += gravityForce;
         if (ground != null)
         {
            CalculateCollisionForce(mass, *ground, CollisionElasticity, PointScaleFactor);
         }

         if (!mass->IsFixed)
         {
            mass->Velocity += (mass->ForceAccumulation / mass->Weight) * deltaTime;
            mass->Position += mass->Velocity * deltaTime;
         }
      }
   }

   vector<mat4> GenerateMassMatrices()
   {
      vector<mat4> matrices;
      matrices.reserve(Masses.size());
      for (const auto& mass : Masses)
      {
         matrices.push_back(translate(mat4{ 1.f }, vec3(mass->Position.x, mass->Position.y, mass->Position.z)));
      }
      return matrices;
   }

   PolyLine<givr::PrimitiveType::LINES> GenerateSpringLines()
   {
      PolyLine<givr::PrimitiveType::LINES> polylines;
      for (const auto& spring : Springs)
      {
         polylines.push_back(Point(spring.Mass1->Position));
         polylines.push_back(Point(spring.Mass2->Position));
      }
      return polylines;
   }
};

struct Controler
{
   MassSpringSystem ToRender;

   int CurrentScene;
   bool IsDragging;

   io::CursorPosition CursorPosition;
   Controler()
   {
      IsDragging = false;
      SetScene(1);
   }
	
   void SetScene(int s)
   {
      CurrentScene = s;
      switch (s)
      {
      case 1:
	      {
	         ToRender = CreateMassSpringLine({ 0,10,0 }, { 0,5,0 }, 2, 1);
	         ToRender.Masses[0]->IsFixed = true;
	         ToRender.SpringKValue = 2.f;
	         ToRender.PointScaleFactor = 0.5f;
	         break;
	      }
      case 2:
	      {
	         ToRender = CreateMassSpringLine({ 0, 20,0 }, { 10,20,0 }, 6, 1);
	         ToRender.Masses[0]->IsFixed = true;
	         ToRender.SpringKValue = 8.f;
	         ToRender.SpringDampeningValue = 0.8f;
	         ToRender.PointScaleFactor = 0.5f;
	         break;
	      }
      case 3:
	      {
	         ToRender = CreateMassSpringRect({ -5,10,-5 }, 10, 10, 10, 4, 1);
	         ToRender.SpringKValue = 200.f;
	         ToRender.SpringDampeningValue = 0.9f;
	         ToRender.PointScaleFactor = 0.25f;
	         float angle = glm::pi<float>() / 12.f;
	         mat4 rotation = rotate(rotate(mat4{ 1.f }, angle, vec3(1, 0, 0)), angle, vec3(0, 0, 1));
	         for (auto& mass : ToRender.Masses)
	         {
	            auto rotated = rotation * vec4(mass->Position.x, mass->Position.y, mass->Position.z, 1);
	            mass->Position = { rotated.x, rotated.y, rotated.z };
	         }
	         break;
	      }
      case 4:
	      {
				int width = 10;
            int height = 15;
	         ToRender = CreateMassSpringCloth({ -7.5f,20,0 }, 15.f, 20.f, width, height, 0.5);
      		// Fix top points
            for (int i = 0; i < width; ++i)
            {
               ToRender.Masses[i]->IsFixed = true;
            }
	         ToRender.SpringKValue = 80.f;
	         ToRender.SpringDampeningValue = 0.8f;
	         ToRender.PointScaleFactor = 0.1f;
            float angle = glm::pi<float>() / 6.f;
            mat4 rotation = rotate(mat4{ 1.f }, angle, vec3(1, 0, 0));
            for (auto& mass : ToRender.Masses)
            {
               auto rotated = rotation * vec4(mass->Position.x, mass->Position.y, mass->Position.z, 1);
               mass->Position = { rotated.x, rotated.y, rotated.z };
            }
	         break;
	      }
      }
   }
};
Controler g_controler;
int main(void)
{
   io::GLFWContext windows;
   auto window = windows.create(io::Window::dimensions{ 1920, 1000 }, "Mass Spring System");
   window.enableVsync(true);

   auto view = View(TurnTable(), Perspective());
   TurnTableControls controls(window, view.camera);
	
   window.keyboardCommands()
      | io::Key(GLFW_KEY_ESCAPE,
         [&](auto const&/*event*/) { window.shouldClose(); })
      | io::Key(GLFW_KEY_1,
         [&](auto const&/*event*/) { g_controler.SetScene(1); })
      | io::Key(GLFW_KEY_2,
         [&](auto const&/*event*/) { g_controler.SetScene(2); })
      | io::Key(GLFW_KEY_3,
         [&](auto const&/*event*/) { g_controler.SetScene(3); })
      | io::Key(GLFW_KEY_4,
         [&](auto const&/*event*/) { g_controler.SetScene(4); });

   window.mouseCommands() | 
      io::MouseButton(     
         GLFW_MOUSE_BUTTON_RIGHT, [&](auto const& event)
         {
   			if (g_controler.CurrentScene == 3)
   			{
		         g_controler.ToRender.Masses[0]->IsFixed = g_controler.IsDragging = event.action == GLFW_PRESS;
   			}
         });

   auto currentCursorCommand = window.cursorCommand();
   window.cursorCommand() = [&](auto const& event)
   {
      if (g_controler.IsDragging && g_controler.CurrentScene == 3)
      {
         float xDiff = (g_controler.CursorPosition.x - event.x) * 40.f / window.width();
         float yDiff = (g_controler.CursorPosition.y - event.y) * 40.f / window.height();
         g_controler.ToRender.Masses[0]->Position.x -= xDiff;
         g_controler.ToRender.Masses[0]->Position.y += yDiff;
      }
      g_controler.CursorPosition = event;
      currentCursorCommand(event);
   };
	

   glClearColor(1.f, 1.f, 1.f, 1.f);

   // rendering style
   auto lineStyle = GL_Line(Colour(0.5, 0.0, 1.0));

   auto sphere = Sphere();
   auto phong = Phong(Colour(0.f, 0.f, 0.f), LightPosition(10.f, 10.f, 10.f));
   
   auto renderSphere = givr::createInstancedRenderable(sphere, phong);

   Plane floor({ 0,-10,0 }, { 0,1,0 });

   Plane* ground = &floor;
   auto green = Phong(Colour(0.2f, 0.8f, 1.f), LightPosition(10.f, 10.f, 10.f));
   auto quad = Quad(Point1(200, -10, 200), Point2(200, -10, -200), Point3(-200, -10, -200), Point4(-200, -10, 200));
   auto groundQuad = givr::createRenderable(quad, green);

   window.run([&](float frameTime)
      {
         view.projection.updateAspectRatio(window.width(), window.height());
         draw(groundQuad, view);

         auto springs = g_controler.ToRender.GenerateSpringLines();
         auto line = givr::createRenderable(springs, lineStyle);
         draw(line, view);


         auto points = g_controler.ToRender.GenerateMassMatrices();
         for (const auto& point : points)
         {
            addInstance(renderSphere, scale(point, vec3(g_controler.ToRender.PointScaleFactor)));
         }
         draw(renderSphere, view);

         // Rough approximation for proper time
         for (int i = 0; i < 16; ++i)
         {
            g_controler.ToRender.DoTimestep(ground);
         }
         
      });

   exit(EXIT_SUCCESS);
}

void CalculateCollisionForce(Mass* mass, Plane ground, float elasticity, float sphereRadius)
{
   // Check if mass has gone past the ground
   vec3f lowestPointOfMass = mass->Position;

   lowestPointOfMass.y -= sphereRadius;
   vec3f lowestPointToOrigin = ground.Position - lowestPointOfMass;
   float cosOfAngle = dot(ground.Normal, lowestPointToOrigin);

   if (cosOfAngle >= 0)
   {
      // Adjust position if needed
      vec3f n = ground.Normal;
   	if (VecIsZero(n))
   	{
         throw std::runtime_error("Normal vector of ground plane is zero.");
   	}
      vec3f positionAdjustment = dot(n, lowestPointToOrigin) / dot(n, n) * n;
      mass->Position += positionAdjustment;
   	
      // Adjust momentum after collision
      float speed = length(mass->Velocity);
      vec3f momentum = mass->Weight * mass->Velocity;
      vec3f normalMomentum = normalize(momentum);
      vec3f newDirection = normalMomentum - 2.f * ground.Normal * dot(normalMomentum, ground.Normal);
      mass->Velocity = elasticity * speed * newDirection;
   }
}

MassSpringSystem CreateMassSpringLine(vec3f origin, vec3f extent, int masses, float massWeight)
{
   MassSpringSystem line;
   for (int i = 0; i < masses; ++i)
   {
      float frac = (float)i / (float)(masses - 1);
      vec3f pos = (1.f - frac) * origin + frac * extent;
      line.AddMass(new Mass(pos, massWeight));
      if (i != 0)
      {
         line.AddSpring(i - 1, i);
      }
   }
   return line;
}

uint Flatten(int i, int j, int k, int indicesPerAxis)
{
   return (uint)(i + indicesPerAxis * (j + indicesPerAxis * k));
}

MassSpringSystem CreateMassSpringRect(vec3f origin, float widthX, float heightY, float depthZ, int massesPerAxis, float massWeight)
{
	if (massesPerAxis < 2)
	{
      throw std::runtime_error("Need more than 1 mass per axis for rects.");
	}
   MassSpringSystem rect;

   rect.Masses.reserve(pow(massesPerAxis, 3));
   for (int j = 0; j < massesPerAxis; ++j)
   {
      float fracY = (float)j / (float)(massesPerAxis - 1);
      for (int k = 0; k < massesPerAxis; ++k)
      {
         float fracZ = (float)k / (float)(massesPerAxis - 1);
         for (int i = 0; i < massesPerAxis; ++i)
         {
            float fracX = (float)i / (float)(massesPerAxis - 1);
            vec3f point = origin;
            point.x += fracX * widthX;
            point.y += fracY * heightY;
            point.z += fracZ * depthZ;
            rect.AddMass(new Mass(point, massWeight));
         }
      }
   }
   float distanceToAttach = length(rect.Masses[Flatten(0, 0, 0, massesPerAxis)]->Position - rect.Masses[Flatten(1, 1, 1, massesPerAxis)]->Position);
   for (int i = 0; i < rect.Masses.size(); ++i)
   {
      for (int j = 0; j < rect.Masses.size(); ++j)
      {
      	// Only connect when i < j to avoid duplicates
      	if (i == j || i > j)
      	{
            continue;
      	}
         auto mass1 = rect.Masses[i];
         auto mass2 = rect.Masses[j];
         auto distance = length(mass1->Position - mass2->Position);
         if (distance <= distanceToAttach)
         {
            rect.AddSpring(i, j);
         }
      }
   }
   return rect;
}

MassSpringSystem CreateMassSpringCloth(vec3f origin, float widthX, float heightY, int xMasses, int yMasses, float massWeight)
{
   if (xMasses < 3 || yMasses < 3)
   {
      throw std::runtime_error("Need more than 3 mass per axis for cloths.");
   }
   MassSpringSystem cloth;
   for (int j = 0; j < yMasses; ++j)
   {
      float fracY = (float)j / (float)(yMasses - 1);
      for (int i = 0; i < xMasses; ++i)
      {
         float fracX = (float)i / (float)(xMasses - 1);
         vec3f point = origin;
         point.x += fracX * widthX;
         point.y -= fracY * heightY;
         cloth.AddMass(new Mass(point, massWeight));
      }
   }
	
   float distanceToAttach = length(cloth.Masses[0]->Position - cloth.Masses[2]->Position);
   for (int i = 0; i < cloth.Masses.size(); ++i)
   {
      for (int j = 0; j < cloth.Masses.size(); ++j)
      {
         // Only connect when i < j to avoid duplicates
         if (i == j || i > j)
         {
            continue;
         }
         auto mass1 = cloth.Masses[i];
         auto mass2 = cloth.Masses[j];
         auto distance = length(mass1->Position - mass2->Position);
         if (distance <= distanceToAttach)
         {
            cloth.AddSpring(i, j);
         }
      }
   }
   return cloth;
}

bool FloatEquals(float lhs, float rhs)
{
   const float epsilon = 0.000001f;
   return abs(lhs - rhs) < epsilon;
}

bool VecIsZero(vec3f vec)
{
   return FloatEquals(vec.x, 0.f) && FloatEquals(vec.y, 0.f) && FloatEquals(vec.z, 0.f);
}

