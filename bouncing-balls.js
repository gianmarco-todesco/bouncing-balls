"use strict";

// the wall is defined by a point and a direction
// the direction indicate the wall outside (i.e. balls can move only on that part) 
class Wall {
    constructor(x,y,ux,uy) {
        this.pos = createVector(x,y);
        this.u = createVector(ux,uy).normalize();
    }
}

// the ball has a radius, a mass (proportional to the squred radius) and a color
// They have also a position and a speed (pixel/millisecond)
class Ball {
    constructor(x,y,r) {
        this.pos = createVector(x,y);
        this.speed = createVector(0,0);
        this.r = r;
        this.mass = r*r;
        let h = floor(r*10);
        this.color = color('hsl('+h+', 100%, 50%)');
        this.cache_valid = false;
    }

    draw() {
        stroke('black')
        fill(this.color)
        circle(this.pos.x,this.pos.y, this.r*2)
    }

    move(dt) {
        this.pos.add(p5.Vector.mult(this.speed,dt));
    }
}

// Abstract collision. it knows only the time to the collision (in milliseconds)
class Collision {
    constructor(t) { this.t = t; }
}

// BallBallCollision
class BallBallCollision extends Collision {
    constructor(t, ball1, ball2) { super(t); this.ball1 = ball1; this.ball2 = ball2;}

    collide() {
        let V = p5.Vector;

        let ball1 = this.ball1, ball2 = this.ball2;
        let totalMass = ball1.mass + ball2.mass;
        let k1 = ball1.mass/totalMass, k2 = ball2.mass/totalMass;
        // versore lungo la direzione di contatto
        let e0 = V.sub(ball2.pos, ball1.pos).normalize();
        let massCenterSpeed = V.add(V.mult(ball1.speed, k1), V.mult(ball2.speed, k2));
        let v1 = V.sub(ball1.speed, massCenterSpeed);
        let v2 = V.sub(ball2.speed, massCenterSpeed);
        // rifletto le velocità (nel sistema di rif. del centro di massa) rispetto al versore e0
        v1 = V.sub(v1, V.mult(e0, 2*v1.dot(e0)));
        v2 = V.sub(v2, V.mult(e0, 2*v2.dot(e0)));
        
        ball1.speed = v1.add(massCenterSpeed);
        ball2.speed = v2.add(massCenterSpeed);
        ball1.cache_valid = ball2.cache_valid = false;
    }

    static computeCollision(t0,ball1, ball2) {
        let v = p5.Vector.sub(ball2.speed, ball1.speed);
        let p = p5.Vector.sub(ball2.pos, ball1.pos);
        let d2 = pow(ball1.r + ball2.r,2);
        // d2 = <p+t*v,p+t*v> => <v,v>t^2 + 2<p,v>t + <p,p> - d2 = 0
        let v2 = v.dot(v), pv = p.dot(v), p2 = p.dot(p);
        let dsc = pow(pv,2) - v2*(p2-d2)
        if(dsc<=0.0) return null;
        let q = sqrt(dsc);
        let dt = (-pv-q)/v2;
        if(dt<0) return null;
        return new BallBallCollision(t0 + dt,ball1, ball2);
    }
}


// BallWallCollision
class BallWallCollision extends Collision {
    constructor(t, ball, wall) { super(t); this.ball = ball; this.wall = wall; }

    collide() {
        let V = p5.Vector;
        let ball = this.ball, wall = this.wall;
        ball.speed = V.sub(ball.speed, V.mult(wall.u, 2*ball.speed.dot(wall.u)));
        ball.cache_valid = false;
    }

    static computeCollision(t0, ball, wall) {
        // <p+t*v-wall.pos, wall.u> - ball.r >= 0
        // <p-wall.pos, wall.u> - ball.r + t*<v,wall.u> >= 0 
        let V = p5.Vector;
        let q = V.sub(ball.pos, wall.pos).dot(wall.u) - ball.r;
        if(q < 0.0) return null; // sono già dentro il muro        
        let v = ball.speed.dot(wall.u);
        if(v >= 0.0) return null; // non mi sto avvicinando
        let dt = -q/v;
        return new BallWallCollision(t0+dt,ball, wall);
    }
}

// the model contains balls and walls and implements the general logic
class Model {
    constructor() {
        this.balls = [];
        this.walls = [
            new Wall(0,0,1,0), 
            new Wall(0,0,0,1),
            new Wall(width,0,-1,0),
            new Wall(0,height,0,-1)
        ];
        this.tLast = millis();
        this.nextCollision = null;
        this.stepTime = 0;
        this.cache = {};
        this.cache_enabled = true;
    }

    draw() { this.balls.forEach(ball => ball.draw()); }
    moveBallsUntil(tCur) {
        if(tCur > this.tLast) {
            let dt = tCur - this.tLast;
            this.balls.forEach(ball => ball.move(dt));
        }
        this.tLast = tCur;
    }

    start() {
        this.tLast = millis();
        this.computeNextCollision();
        this.balls.forEach(ball=>ball.cache_valid = true);
        console.log(this.balls.length + " balls, " + this.walls.length + " walls");        
    }

    step() {
        let tCur = millis();
        if(this.nextCollision !== null) {
            let guard = 0; // fidarsi è bene.... 
            let collision = this.nextCollision;
            while(collision !== null && collision.t <= tCur) {
                this.moveBallsUntil(collision.t);
                collision.collide();
                this.computeNextCollision();
                collision = this.nextCollision;
                guard++;
                if(guard > 200) { console.warn("Too many collisions per frame"); return; }
            }
        }
        this.moveBallsUntil(tCur);
        this.stepTime = millis() - tCur;
    }

    computeNextCollision() {
        let t = this.tLast;
        let nextCollision = null;
        for(let i=0; i<this.balls.length; i++) {
            let ball_i = this.balls[i];
            for(let j=i+1; j<this.balls.length; j++) {
                let ball_j = this.balls[j];
                let cname = 'b'+i+'_'+j;
                let c;
                if(this.cache_enabled && ball_i.cache_valid && ball_j.cache_valid && cname in this.cache) 
                    c = this.cache[cname];
                else 
                    c = this.cache[cname] = BallBallCollision.computeCollision(t, ball_i, ball_j);
                if(c && (nextCollision == null || nextCollision.t > c.t)) {
                    nextCollision = c;
                }
            }
            this.walls.forEach((wall,j) => {
                let cname = 'w'+i+'_'+j;
                let c;
                if(this.cache_enabled && ball_i.cache_valid  && cname in this.cache) 
                    c = this.cache[cname];
                else 
                    c = this.cache[cname] = BallWallCollision.computeCollision(t, ball_i, wall);
                if(c && (nextCollision == null || nextCollision.t > c.t)) {
                    nextCollision = c;
                }
            })
            ball_i.cache_valid = true;
        }
        this.nextCollision = nextCollision;
    }
}

// a simple model, with just two balls
function createSimpleModel() {
    let model = new Model();
    let ball;

    ball = new Ball(200,200, 30);
    ball.speed.set(0.1,-0.03);
    model.balls.push(ball);          

    ball = new Ball(600,200, 70);
    ball.speed.set(-0.1,-0.03);
    model.balls.push(ball);     

    model.start();
    return model;
}

// standard model, with many balls with different sizes and colors
function createModel() {
    let model = new Model();
    let d = 40, mrg = 10;
    let maxSpeed = 0.03;
    let rMin = d/8, rMax = d/2;
    let nx = floor((width-mrg)/d), ny = floor((height-mrg)/d);
    for(let i=0; i<ny; i++) {
        for(let j=0; j<nx; j++) {
            let x = (width-nx*d)/2+(j+0.5)*d;
            let y = (height-ny*d)/2+(i+0.5)*d;
            let ball = new Ball(x,y, rMin + pow(random(),3)*(rMax-rMin));
            let speed = random() * maxSpeed;
            let speedPhi = random() * 2.0 * PI;
            ball.speed.set(cos(speedPhi)*speed, sin(speedPhi)*speed);
            model.balls.push(ball);          
        }
    }
    model.start();
    return model;
}

// main program
let model;

function setup() {
    createCanvas(800, 600);
    model = createModel();
}

function draw() {
    background(200);
    model.draw();
    model.step();
}

