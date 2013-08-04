package com.tvganesh.robotwalker;

/* Designed and developed by Tinniam V Ganesh, 4 Aug 2013
 * Uses Box2D physics engine
 * Uses AndEngine
 */
import java.util.Timer;
import java.util.TimerTask;
import org.andengine.engine.camera.Camera;
import org.andengine.engine.options.EngineOptions;
import org.andengine.engine.options.ScreenOrientation;
import org.andengine.engine.options.resolutionpolicy.RatioResolutionPolicy;
import org.andengine.entity.primitive.Rectangle;
import org.andengine.entity.scene.IOnSceneTouchListener;
import org.andengine.entity.scene.Scene;
import org.andengine.entity.scene.background.Background;
import org.andengine.entity.sprite.Sprite;
import org.andengine.entity.text.Text;
import org.andengine.entity.text.TextOptions;
import org.andengine.entity.util.FPSLogger;
import org.andengine.extension.physics.box2d.PhysicsConnector;
import org.andengine.extension.physics.box2d.PhysicsFactory;
import org.andengine.extension.physics.box2d.PhysicsWorld;
import org.andengine.extension.physics.box2d.util.Vector2Pool;
import org.andengine.input.sensor.acceleration.AccelerationData;
import org.andengine.input.sensor.acceleration.IAccelerationListener;
import org.andengine.input.touch.TouchEvent;
import org.andengine.opengl.font.Font;
import org.andengine.opengl.texture.TextureOptions;
import org.andengine.opengl.texture.atlas.bitmap.BitmapTextureAtlas;
import org.andengine.opengl.texture.atlas.bitmap.BitmapTextureAtlasTextureRegionFactory;
import org.andengine.opengl.texture.region.TextureRegion;
import org.andengine.opengl.vbo.VertexBufferObjectManager;
import org.andengine.ui.activity.SimpleBaseGameActivity;
import org.andengine.util.HorizontalAlign;

import android.hardware.SensorManager;
import android.util.Log;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef.BodyType;
import com.badlogic.gdx.physics.box2d.joints.DistanceJointDef;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJoint;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;
import com.badlogic.gdx.physics.box2d.joints.WeldJointDef;
import com.badlogic.gdx.physics.box2d.FixtureDef;



public class RobotWalker extends SimpleBaseGameActivity implements  IAccelerationListener, IOnSceneTouchListener {
	private static final int CAMERA_WIDTH = 720;
	private static final int CAMERA_HEIGHT = 480;
	public static final float PIXEL_TO_METER_RATIO_DEFAULT = 32.0f;
	
    
    private Scene mScene;
    
    private PhysicsWorld mPhysicsWorld;
    
	private BitmapTextureAtlas mBitmapTextureAtlas;   
	private TextureRegion mWallTextureRegion;
	private TextureRegion mLegTextureRegion;
	private TextureRegion mRobotTextureRegion;
	
	Rectangle r;
	Rectangle ground,roof,left,right;
	static Sprite lWall,rWall;
	Body lWallBody,rWallBody;
	Sprite wheel1,wheel2;
	Body wheelBody1,wheelBody2;
	Sprite car;
	Body  carBody;
	Sprite upperLeg,lowerLeg;
	Body upperLegBody,lowerLegBody;
	Sprite leg1,leg2,leg3,leg4,leg5,leg6,leg7,leg8;
	Body legBody1,legBody2,legBody3,legBody4,legBody5;
	Sprite robot;
	Body robotBody;
	static Font mFont;
	static Text bText;
	
	final FixtureDef gameFixtureDef = PhysicsFactory.createFixtureDef(10f, 0.0f, 0.0f);

    private static FixtureDef BODY_FIXTURE_DEF = PhysicsFactory.createFixtureDef(1f, 0.0f, 0.5f);
    private static FixtureDef LEG_FIXTURE_DEF = PhysicsFactory.createFixtureDef(3f, 0.0f, 0.5f);
	static RevoluteJoint rJoint;
	
	public EngineOptions onCreateEngineOptions() {
		
		final Camera camera = new Camera(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT);
		return new EngineOptions(true, ScreenOrientation.LANDSCAPE_FIXED, new RatioResolutionPolicy(CAMERA_WIDTH, CAMERA_HEIGHT), camera);
	}
	
	public void onCreateResources() {
		BitmapTextureAtlasTextureRegionFactory.setAssetBasePath("gfx/");	
		this.mBitmapTextureAtlas = new BitmapTextureAtlas(this.getTextureManager(), 165, 545, TextureOptions.BILINEAR);		
		
		this.mRobotTextureRegion = BitmapTextureAtlasTextureRegionFactory.createFromAsset(this.mBitmapTextureAtlas, this, "robot.png", 0, 0);		
		this.mLegTextureRegion = BitmapTextureAtlasTextureRegionFactory.createFromAsset(this.mBitmapTextureAtlas, this, "leg.png", 150, 15);
		this.mWallTextureRegion = BitmapTextureAtlasTextureRegionFactory.createFromAsset(this.mBitmapTextureAtlas, this, "wall.png", 160, 65);		
		this.mBitmapTextureAtlas.load();
		
	
	}
	
	@Override
	public Scene onCreateScene() {
		this.mEngine.registerUpdateHandler(new FPSLogger());

		this.mScene = new Scene();
		this.mScene.setBackground(new Background(0.09804f, 0.6274f, 0.8784f));
		this.mScene.setOnSceneTouchListener(this);
		this.mPhysicsWorld = new PhysicsWorld(new Vector2(0, SensorManager.GRAVITY_MOON), false);

		this.robotWalk(mScene);
		this.mScene.registerUpdateHandler(this.mPhysicsWorld);
	
		return mScene;		
		
	}
	
	public void robotWalk(Scene mScene){
		//Create the floor		
		final VertexBufferObjectManager vertexBufferObjectManager = this.getVertexBufferObjectManager();
		ground = new Rectangle(0, CAMERA_HEIGHT - 2, CAMERA_WIDTH, 2, vertexBufferObjectManager);
		roof = new Rectangle(0, 0, CAMERA_WIDTH, 2, vertexBufferObjectManager);
		left = new Rectangle(0, 0, 2, CAMERA_HEIGHT, vertexBufferObjectManager);
		right = new Rectangle(CAMERA_WIDTH - 2, 0, 2, CAMERA_HEIGHT, vertexBufferObjectManager);

		// Create the ground, roof and the walls
		final FixtureDef wallFixtureDef = PhysicsFactory.createFixtureDef(0, 0.0f, 0.2f);
		PhysicsFactory.createBoxBody(this.mPhysicsWorld, ground, BodyType.StaticBody, wallFixtureDef);
		PhysicsFactory.createBoxBody(this.mPhysicsWorld, roof, BodyType.StaticBody, wallFixtureDef);
		PhysicsFactory.createBoxBody(this.mPhysicsWorld, left, BodyType.StaticBody, wallFixtureDef);		
		PhysicsFactory.createBoxBody(this.mPhysicsWorld, right, BodyType.StaticBody, wallFixtureDef);
       
        
		this.mScene.attachChild(ground);
		this.mScene.attachChild(roof);
		this.mScene.attachChild(left);
		this.mScene.attachChild(right);

		// Create the left wall - Collisions happen between bodies
		lWall = new Sprite(0, 0, this.mWallTextureRegion, this.getVertexBufferObjectManager());
		lWallBody = PhysicsFactory.createCircleBody(this.mPhysicsWorld, lWall, BodyType.StaticBody, wallFixtureDef);
		this.mPhysicsWorld.registerPhysicsConnector(new PhysicsConnector(lWall, lWallBody, true, true));
		this.mScene.attachChild(lWall);
		
		// Create right wall - Collisions happen between bodies
		rWall = new Sprite(715, 0, this.mWallTextureRegion, this.getVertexBufferObjectManager());
		rWallBody = PhysicsFactory.createCircleBody(this.mPhysicsWorld, rWall, BodyType.StaticBody, wallFixtureDef);
		this.mPhysicsWorld.registerPhysicsConnector(new PhysicsConnector(rWall, rWallBody, true, true));
		this.mScene.attachChild(rWall);
			
		//Create the robot body
		robot = new Sprite(100, 360, this.mRobotTextureRegion, this.getVertexBufferObjectManager());
		robotBody = PhysicsFactory.createBoxBody(this.mPhysicsWorld, robot, BodyType.DynamicBody, BODY_FIXTURE_DEF);
		this.mPhysicsWorld.registerPhysicsConnector(new PhysicsConnector(robot, robotBody, true, true));
		this.mScene.attachChild(robot);
			
		//Create 6 different legs spaced apart
		int secs = 1;
		createLeg(100,360,secs);
		createLeg(120,360,secs);
		createLeg(140,360,secs);
		createLeg(160,360,secs);
		createLeg(180,360,secs);
		createLeg(200,360,secs);
		createLeg(220,360,secs);
		createLeg(240,360,secs);
		
	
		this.mScene.registerUpdateHandler(this.mPhysicsWorld);
	}
	

	@Override
	public void onAccelerationAccuracyChanged(AccelerationData pAccelerationData) {
		// TODO Auto-generated method stub
		
	}	

	@Override
	public void onAccelerationChanged(AccelerationData pAccelerationData) {
		final Vector2 gravity = Vector2Pool.obtain(pAccelerationData.getX(), pAccelerationData.getY());
		this.mPhysicsWorld.setGravity(gravity);
		Vector2Pool.recycle(gravity);
		
	}


	@Override
	public void onResumeGame() {
		super.onResumeGame();

		this.enableAccelerationSensor(this);

	}

	@Override
	public void onPauseGame() {
		super.onPauseGame();

		this.disableAccelerationSensor();
	}
	
	
	@Override
	public boolean onSceneTouchEvent(Scene pScene, TouchEvent pSceneTouchEvent) {
		if(this.mPhysicsWorld != null) {
			if(pSceneTouchEvent.isActionDown()) {
				//this.addBall(pSceneTouchEvent.getX(), pSceneTouchEvent.getY());
				return true;
			}
		}
		return false;
	}

    //Create the upper and lower part of the leg
    public void createLeg(float x, float y, int secs ){
    	// Create upper leg
		upperLeg = new Sprite(x, y, this.mLegTextureRegion, this.getVertexBufferObjectManager());
		upperLegBody = PhysicsFactory.createBoxBody(this.mPhysicsWorld, upperLeg, BodyType.DynamicBody, LEG_FIXTURE_DEF);
		this.mPhysicsWorld.registerPhysicsConnector(new PhysicsConnector(upperLeg, upperLegBody, true, true));
		this.mScene.attachChild(upperLeg);
		
		//Create an anchor/pivot at the body of the robot
		Vector2 anchor1 = new Vector2(x/PIXEL_TO_METER_RATIO_DEFAULT,y/PIXEL_TO_METER_RATIO_DEFAULT);
		
		//Attach upper leg to the body using a revolute joint with a motor
        final RevoluteJointDef rJointDef = new RevoluteJointDef();	    
        rJointDef.initialize(upperLegBody, robotBody, anchor1);
        Log.d("x","lc :" + robotBody.getLocalCenter()+ "position:"+ robotBody.getPosition());
        rJointDef.enableMotor = true;
        rJointDef.enableLimit = true;
		rJoint = (RevoluteJoint) this.mPhysicsWorld.createJoint(rJointDef);		
		rJoint.setMotorSpeed(2);
		rJoint.setMaxMotorTorque(10);
		//Set upper and lower limits for the swing of the leg
		rJoint.setLimits((float)(0 * (Math.PI)/180), (float)(50 * (Math.PI)/180));		
		new IntervalTimer(secs,rJoint);		
		
		// Create lower leg
		lowerLeg= new Sprite(x, (y+50), this.mLegTextureRegion, this.getVertexBufferObjectManager());
		lowerLegBody = PhysicsFactory.createBoxBody(this.mPhysicsWorld, lowerLeg, BodyType.DynamicBody, LEG_FIXTURE_DEF);
		this.mPhysicsWorld.registerPhysicsConnector(new PhysicsConnector(lowerLeg, lowerLegBody, true, true));
		this.mScene.attachChild(lowerLeg);
		
		// Connect the lower and upper leg with distance joint
		Vector2 anchor2 = new Vector2(x/PIXEL_TO_METER_RATIO_DEFAULT, (y+50)/PIXEL_TO_METER_RATIO_DEFAULT);
		//Create a distanceJoint between upper & lower leg
		DistanceJointDef distanceJoint1 = new DistanceJointDef();
		distanceJoint1.initialize(upperLegBody,lowerLegBody, anchor2,anchor2);		   
		distanceJoint1.collideConnected = true;
		distanceJoint1.dampingRatio = 0.5f;
		distanceJoint1.frequencyHz = 10.0f;
		this.mPhysicsWorld.createJoint(distanceJoint1);
	
    	
    }
}	

//Create a timer that fires every interval and reverses motor to simulate the motion
class IntervalTimer {
    Timer timer;
   

    public IntervalTimer(int seconds, RevoluteJoint rj) {
    	Log.d("Inside","in");
        timer = new Timer();  //At this line a new Thread will be created      
        timer.scheduleAtFixedRate(new RemindTask(rj), seconds*1000, 1000);
        
    }

    class RemindTask extends TimerTask {
    	RevoluteJoint rj1;;
    	RemindTask(RevoluteJoint rj){
    		rj1 = rj;
    	}
        @Override
        public void run() {
        	Log.d("x","x" +"Reversing motor");
        	reverseMotor();
            
        }
        
        public void reverseMotor(){
        	rj1.setMotorSpeed(-(rj1.getMotorSpeed()));
        	rj1.setMaxMotorTorque(10);
        	Log.d("aa","speed:"+rj1.getMotorSpeed() + "torque:" + rj1.getMotorTorque());
        	
        }
    }
   
}