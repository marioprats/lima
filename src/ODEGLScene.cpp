/*
 * Copyright (c) 2007 University of Jaume-I, 2013 Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mario Prats
 */

#include <lima/ODEGLScene.h>
#include <lima/Joint.h>
#ifdef _USE_GL_
#include <GL/glut.h>
#endif

#include <iostream>
using namespace std;

ODEGLScene::ODEGLScene() :
    Scene()
{

#ifdef _USE_GL_
  int argc=0;
  char **argv=NULL;

  glutInit(&argc,argv);
  /* Create an RGBA-mode context */
  GLWorld = OSMesaCreateContext( OSMESA_RGBA, NULL );
#endif
  simulation_started = false;

#ifdef _USE_ODE_
  /* Setup of the ODE world */
  dInitODE();
  ODEWorld = dWorldCreate();
  ODESpace = dHashSpaceCreate(0);
  // Set up gravity force	
  dWorldSetGravity(ODEWorld,0,-9.810 /* m/s^2 */,0);
  // Set ERP
  dWorldSetERP(ODEWorld,0.1);
// 	dWorldSetCFM(ODEWorld,0.008);
  // Create contact group	
  contactgroup=dJointGroupCreate(0);
#endif
}

void ODEGLScene::configure()
{
#ifdef _USE_GL_
  GLfloat light_difuse[] =
  { 0.3, 0.3, 0.3, 0};
  GLfloat light_ambient[] =
  { 0.3, 0.3, 0.3, 0};
  GLfloat light_specular[] =
  { 0.8, 0.8, 0.8, 0.0};
  GLfloat light_position[] =
  { 20.0, 20.0, 10.0, 0.0};
  GLfloat light_position1[] =
  { -20.0, 20.0, -10.0, 0.0};

  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_difuse);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light_difuse);
  glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);

  glEnable(GL_DEPTH_TEST);
//   	glDepthFunc(GL_LESS);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_SMOOTH);
//   	glEnable(GL_RESCALE_NORMAL);
  glEnable(GL_NORMALIZE);
  glClearColor(0, 0, 0, 1.0);
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
#endif
}

#ifdef _USE_ODE_
void configureDynamics(dWorldID *ODEWorld, dSpaceID *ODESpace, vpHomogeneousMatrix wMo, Object *o)
{
  if (o!=NULL && o->model!=NULL && o->model->dynamics!=NULL)
  {
    //the object has a dynamic model.
    //we must initialize body and geom position
    o->model->dynamics->createBody(ODEWorld);
    o->model->dynamics->createGeom(ODESpace);
    o->model->dynamics->setPose(wMo);

    if (o->linkedBy!=NULL && o->linkedBy->dynamic_model!=NULL)
    {
      //Initialize the joint that links this object with his parent
      o->linkedBy->dynamic_model->createJoint(ODEWorld);
      o->linkedBy->dynamic_model->configureJoint(o->linkedBy);
    }
    for (unsigned int i=0; i<o->linkedTo.size(); i++)
    {
      configureDynamics(ODEWorld, ODESpace, wMo*o->linkedTo[i]->getNextObjectPose(),o->linkedTo[i]->next);
    }
  }
}

void ODEGLScene::addObject(vpHomogeneousMatrix wMo, Object *o)
{
  Scene::addObject(wMo,o);
  configureDynamics(&ODEWorld, &ODESpace, wMo,o);
}

void ODEGLScene::handleCollisionBetween(dGeomID o0, dGeomID o1)
{
  // Create an array of dContact objects to hold the contact joints
  static const int MAX_CONTACTS = 10;
  dContact contact[MAX_CONTACTS];

  for (int i = 0; i < MAX_CONTACTS; i++)
  {
    contact[i].surface.mode = dContactBounce | dContactSoftCFM| dContactSlip1 |dContactSlip2;
    contact[i].surface.bounce = 0.6;
    contact[i].surface.bounce_vel = (dReal) 0.01;
    contact[i].surface.soft_cfm = 0.015;
    //instead of using friction, which makes the simulation slow,
    //we set slip coefficients
    //This is faster, but the behavior is not real.
    //TODO: For contacts with the floor, slip should not be used. Detect them. It could be done by comparing the current colliding geoms with all the object geoms defined in the world. Then, add a flag to an object specifying if it is floor.
    contact[i].surface.mu = dInfinity;
    contact[i].surface.slip1 = (dReal) 0.002;
    contact[i].surface.slip2 = (dReal) 0.002;
  }
  if (int numc = dCollide(o0, o1, MAX_CONTACTS, &contact[0].geom, sizeof(dContact)))
  {
    dBodyID b1 = dGeomGetBody(o0);
    dBodyID b2 = dGeomGetBody(o1);
    for (int i = 0; i < numc; i++)
    {
      dJointID c = dJointCreateContact(ODEWorld, contactgroup, contact + i);
      dJointAttach(c, b1, b2);
    }
  }
}

void nearCallback(void *data, dGeomID o0, dGeomID o1)
{
  ((ODEGLScene*)data)->handleCollisionBetween(o0,o1);
}

//Updates object position according to the simulation state
void updateObjectPositionBySimulation(Object *o)
{
  if (o!=NULL && o->model->dynamics!=NULL && o->model->dynamics->geom_initialized)
  {
    //update the position of the object
    //this updates wMo attribute of the Object class
    o->setPosition(o->model->dynamics->getPose());
    if (o->linkedBy!=NULL && o->linkedBy->dynamic_model!=NULL)
    {
      //update the joint that contains this object
      //this updates the particular Joint object parameters (current_value,etc)
      o->linkedBy->dynamic_model->updateJointBySimulation(o->linkedBy);
    }
    //Do it for childs recursively
    for (unsigned int i=0; i<o->linkedTo.size(); i++)
    {
      updateObjectPositionBySimulation(o->linkedTo[i]->next);
    }
  }
}
#else
//Updates object position according to the object tree values
void updateObjectPositionBySimulation(vpHomogeneousMatrix wMo, Object *o)
{
  if (o != NULL)
  {

    o->setPosition(wMo);
    for (unsigned int i = 0; i < o->linkedTo.size(); i++)
    {
      updateObjectPositionBySimulation(wMo * o->linkedTo[i]->getNextObjectPose(), o->linkedTo[i]->next);
    }
  }
}
#endif

void ODEGLScene::simulationStep()
{
#ifdef _USE_ODE_
  static float seconds_per_step=0.001f; //1ms: Simulation step
  if (!simulation_started)
  {
    //The first time we run this method: start timing
    gettimeofday(&last,NULL);
    simulation_started=true;
  }

  struct timeval current;
  gettimeofday(&current,NULL);
  float seconds_elapsed=((current.tv_sec*1000000.0+current.tv_usec)-(last.tv_sec*1000000.0+last.tv_usec))/1000000.0;

  unsigned int steps=(unsigned int)(seconds_elapsed/seconds_per_step);

  for ( int i=0; i<steps; i++ )
  {
    //Managing Collisions	
    dSpaceCollide(ODESpace,this,&nearCallback);

    // Step world	
    dWorldStep(ODEWorld, seconds_per_step);

    // Remove all temporary collision joints now that the world has been stepped
    dJointGroupEmpty(contactgroup);
  }

  //Update the objects position according to the dynamic simulation
  for (unsigned i=0; i<objects.size();i++)
  {
    updateObjectPositionBySimulation(objects[i]);
  }

  //Restart clock from now
  gettimeofday(&last,NULL);
#else
  //The absolute position of each subobject is computed knowing the absolute position
  //of the root, and the relative position between object-subobject, given by the object tree
// cerr<< "ODEGLScene::SimulationStep"<<endl;
// cerr<< "ODEGLScene::object.size()="<<objects.size()<<endl;
// cerr<< "ODEGLS-> wMo:  "<< objects[0]->wMo<<endl;
  for (unsigned i = 0; i < objects.size(); i++)
  {
    updateObjectPositionBySimulation(objects[i]->wMo, objects[i]);
  }
// cerr<<"ODEGLScene-->2"<<endl;

#endif
}

ODEGLScene::~ODEGLScene()
{
#ifdef _USE_GL_
  OSMesaDestroyContext( GLWorld );
#endif
#ifdef _USE_ODE_
  dSpaceDestroy(ODESpace);
  dWorldDestroy(ODEWorld);
  dCloseODE();
#endif  
}

