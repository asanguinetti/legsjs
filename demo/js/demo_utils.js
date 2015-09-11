'use strict';

var bind = function(obj, method) {
  return function() {
    method.apply(obj, arguments);
  };
};

var extend = function(extended, base) {
  extended.prototype = Object.create(base.prototype);
  extended.prototype.base = base;
  extended.prototype.constructor = extended;
};

var Demo = function(viewport) {
  this.pinchDist = 0;
  this.inclination = Math.PI/4;
  this.azimuth = 0;
  this.zoom = 50;
  this.cameraPosMat = new THREE.Matrix4();
  this.m1 = new THREE.Matrix4();
  this.m2 = new THREE.Matrix4();
  this.m3 = new THREE.Matrix4();
  this.time_last = Date.now();
  this.time_delta = 0;

  this.scene = new THREE.Scene();

  this.setUpScene(viewport);
  this.setUpPhysics();
};

Demo.prototype.setUpScene = function(viewport) {
  this.scene.renderer = new THREE.WebGLRenderer({antialias: true});
  this.scene.renderer.setClearColor(0xeeeeff, 1);
  this.scene.renderer.setSize(window.innerWidth, window.innerHeight);
  this.scene.renderer.shadowMapEnabled = true;
  this.scene.renderer.shadowMapSoft = true;
  viewport.appendChild(this.scene.renderer.domElement);
  this.scene.renderer.domElement.addEventListener('touchstart', bind(this, this.onTouchStart));
  this.scene.renderer.domElement.addEventListener('touchmove', bind(this, this.onMouseMove));
  this.scene.renderer.domElement.addEventListener("mousewheel", bind(this, this.onMouseWheel), false);
  this.scene.renderer.domElement.addEventListener("DOMMouseScroll", bind(this, this.onMouseWheel), false);
  this.scene.renderer.domElement.addEventListener('mousemove', bind(this, this.onMouseMove));
  this.scene.renderer.domElement.addEventListener('mousedown', bind(this, this.onMouseDown));
  this.scene.renderer.domElement.addEventListener('mouseup', bind(this, this.onMouseUp));

  this.scene.camera = new THREE.PerspectiveCamera(
    35,
    window.innerWidth / window.innerHeight,
    1,
    1000
  );
  this.scene.camera.position.set(60, 60, 50);
  this.scene.camera.up.set(0,0,1);
  this.scene.camera.lookAt(this.scene.position);
  this.scene.add(this.scene.camera);
  
  // Light
  var light = new THREE.DirectionalLight(0xFFFFFF);
  light.position.set( -15, 20, 40 );
  light.target.position.copy(this.scene.position);
  light.castShadow = true;
  light.shadowCameraLeft = -60;
  light.shadowCameraTop = -60;
  light.shadowCameraRight = 60;
  light.shadowCameraBottom = 60;
  light.shadowCameraNear = 20;
  light.shadowCameraFar = 200;
  light.shadowBias = -.0005
  light.shadowMapWidth = light.shadowMapHeight = 2048;
  light.shadowDarkness = .7;
  this.scene.add(light);

  this.mouseDownPos = new THREE.Vector2();
  this.mouseDragging = false;
};

Demo.prototype.setUpPhysics = function() {
  var collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
  var dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
  var overlappingPairCache = new Ammo.btDbvtBroadphase();
  var solver = new Ammo.btSequentialImpulseConstraintSolver();
  // this.scene.debugDrawer = new DebugDrawer();
  this.scene.world = new Ammo.btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
  // this.scene.world.setDebugDrawer(this.scene.debugDrawer.btDebugDrawer);
  this.scene.world.setGravity(new Ammo.btVector3(0,0,-20));
  this.scene.world.bodies = [];
};

Demo.prototype.onTouchStart = function(event) {
  if(typeof event == 'undefined')
    event = window.event;
  
  if(event.touches) {
    if(event.touches.length == 2) {
      this.pinchDist = vec2.distance([event.touches[0].pageX, 
                                      event.touches[0].pageY], 
                                     [event.touches[1].pageX, 
                                      event.touches[1].pageY]);
      event.preventDefault();
    }
  }
};

Demo.prototype.onMouseMove = function(event) {
  var x;
  var y;

  if(typeof event == 'undefined')
    event = window.event;
  
  event.preventDefault();

  if(event.changedTouches) {
    if(event.touches.length == 2) {
      /* it's a pinch to zoom gesture */
      newDist = vec2.distance([event.touches[0].pageX, 
                               event.touches[0].pageY], 
                              [event.touches[1].pageX, 
                               event.touches[1].pageY]);
      this.zoom += (this.pinchDist - newDist);
      this.pinchDist = newDist;
      this.zoom = Math.max(0, Math.min(this.zoom, 80));
      return;
    }
    event = event.changedTouches[0];
  }

  /* only proceed if this is a drag gesture */
  if(!this.mouseDragging)
    return;
    
  if(event.clientX) {
      x = event.clientX+document.body.scrollLeft;
      y = event.clientY+document.body.scrollTop;
  } else if(event.pageX) {
      x = event.pageX+window.pageXOffset;
      y = event.pageY+window.pageYOffset;
  }

  /* gets the position relative to the position where the drag started */
  x -= this.mouseDownPos.x;
  y -= this.mouseDownPos.y;

  /* updates the mouse down position */
  this.mouseDownPos.x += x;
  this.mouseDownPos.y += y;  

  /* dragging across the whole screen from left to right should add 2PI to the azimuth */
  this.azimuth += x * 2 * Math.PI / window.innerWidth;

  /* dragging across the whole screen from top to bottom should subtract PI/2 to the inclination */
  this.inclination -= y * Math.PI / (2*window.innerHeight);

  /* clamps the inclination to avoid gimbal lock problems */
  this.inclination = Math.min(Math.max(this.inclination, -Math.PI/2), Math.PI/2);
};

Demo.prototype.onMouseWheel = function(event) {
  /* cross-browser wheel delta */
  var event = window.event || event; /* IE */
  var delta = Math.max(-1, Math.min(1, (event.wheelDelta || -event.detail)));
  this.zoom += delta;
  this.zoom = Math.max(0, Math.min(this.zoom, 80));
};

Demo.prototype.onMouseDown = function(event) {
  if(typeof event == 'undefined')
    event = window.event;
  
  event.preventDefault();

  if(event.changedTouches) {
    if(event.touches.length != 1)
      return;
    event = event.changedTouches[0];
  }
    
  if(event.clientX) {
    this.mouseDownPos.x = event.clientX+document.body.scrollLeft;
    this.mouseDownPos.y = event.clientY+document.body.scrollTop;
  } else if(event.pageX) {
    this.mouseDownPos.x = event.pageX+window.pageXOffset;
    this.mouseDownPos.y = event.pageY+window.pageYOffset;
  }

  this.mouseDragging = true;
};

Demo.prototype.onMouseUp = function(event) {
  this.mouseDragging = false;
};

Demo.prototype.updateCamera = function()
{
  /* start with the camera at the origin */
  this.scene.camera.position.copy(this.scene.position);

  /* rotates the camera position around the Z axis for the azimuth */
  this.m1.makeRotationZ(this.azimuth);

  /* raises the camera rotating around the X axis for the inclination */
  this.m2.makeRotationX(-this.inclination);

  /* moves the camera away from the center */
  this.m3.makeTranslation(0, -this.zoom, 0);

  this.cameraPosMat.multiplyMatrices(this.m1, this.m2);
  this.cameraPosMat.multiply(this.m3);

  /* applies the transformation to the camera position */
  this.scene.camera.position.applyMatrix4(this.cameraPosMat);
  this.scene.camera.position.add(this.scene.camera.target.visual.position);
  this.scene.camera.position.z = Math.max(this.scene.camera.position.z, 1);
  this.scene.camera.lookAt(this.scene.camera.target.visual.position);
};

Demo.prototype.updatePhysics = function() {
    /* steps world */
    this.time_delta = Date.now() - this.time_last;
    this.scene.world.stepSimulation(this.time_delta / 1000.0, 1500, 1 / 2000);
    this.time_last += this.time_delta;
};

Demo.prototype.updateVisuals = function() {
  /* Read position data into visuals */
  this.scene.world.bodies.forEach(function(b) {
    b.updateVisual();
  });
  // this.scene.world.debugDrawWorld();
  // this.scene.debugDrawer.clear();
};

Demo.prototype.animate = function() {
  requestAnimationFrame(bind(this, this.animate));
  this.updateCamera();
  this.updateVisuals();
  this.updatePhysics();
  this.scene.renderer.clear();
  this.scene.renderer.render(this.scene, this.scene.camera);
};

var Force = function() {
  this.arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0),
                                     new THREE.Vector3(0, 0, 0),
                                     0);
};

Force.prototype.applyTo = function(body, force, relPos) {
  body.toWorldFrame(relPos, this.arrow.position);
  this.arrow.setLength(force.length());
  this.arrow.setDirection(force.clone().normalize());
  body.applyForce(force, relPos);
};

Force.prototype.show = function() {
  this.arrow.visible = true;
};

Force.prototype.hide = function() {
  this.arrow.visible = false;
};

Force.prototype.buildAndInsert = function(scene) {
  scene.add(this.arrow);
};

var DebugDrawer = function() {
  var wrappedFrom;
  var wrappedTo;
  var wrappedColor;
  var line;

  this.debugMode = 0 | (1 << 11) | (1 << 12);
  this.objIndex = 0;
  this.objects = [];
  for(var i = 0; i < 1500; i++)
  {
    this.objects.push(new THREE.Line(new THREE.Geometry(), 
                                     new THREE.LineBasicMaterial({color: new THREE.Color()})));
    this.objects[i].geometry.vertices.push(new THREE.Vector3());
    this.objects[i].geometry.vertices.push(new THREE.Vector3());
  }
  this.btDebugDrawer = new Ammo.DebugDraw();
  this.btDebugDrawer.userData = this;
  this.btDebugDrawer.drawLine = function(from, to, color) {
    wrappedFrom = Ammo.wrapPointer(from, Ammo.btVector3);
    wrappedTo = Ammo.wrapPointer(to, Ammo.btVector3);
    wrappedColor = Ammo.wrapPointer(color, Ammo.btVector3);

    line = this.userData.objects[this.userData.objIndex++];
    line.material.color.setRGB(wrappedColor.x(), wrappedColor.y(), wrappedColor.z());
    line.geometry.vertices[0].set(wrappedFrom.x(), wrappedFrom.y(), wrappedFrom.z());
    line.geometry.vertices[1].set(wrappedTo.x(), wrappedTo.y(), wrappedTo.z());
    line.geometry.verticesNeedUpdate = true;

    scene.add(line);
  };
  this.btDebugDrawer.drawContactPoint = function(PointOnB, normalOnB, distance, lifeTime, color) {
    // TODO: implement
  };
  this.btDebugDrawer.reportErrorWarning = function(warningString) {
    console.log(warningString);
  };
  this.btDebugDrawer.draw3dText = function(location, textString) {
    // TODO: implement
  };
  this.btDebugDrawer.setDebugMode = function(debugMode) {
    this.userData.debugMode = debugMode;
  };
  this.btDebugDrawer.getDebugMode = function() {
    return this.userData.debugMode;
  };
};

DebugDrawer.prototype.clear = function() {
  this.objIndex = 0;
};