import type { ICrowd, IAgentParameters, INavMeshParameters, IObstacle, INavigationEnginePlugin } from "@babylonjs/core/Navigation/INavigationEngine";
import { Logger } from "@babylonjs/core/Misc/logger";
import { VertexData } from "@babylonjs/core/Meshes/mesh.vertexData";
import { Mesh } from "@babylonjs/core/Meshes/mesh";
import type { Scene } from "@babylonjs/core/scene";
import { Epsilon, Vector3, Matrix } from "@babylonjs/core/Maths/math";
import type { TransformNode } from "@babylonjs/core/Meshes/transformNode";
import type { Observer } from "@babylonjs/core/Misc/observable";
import { Observable } from "@babylonjs/core/Misc/observable";
import type { Nullable } from "@babylonjs/core/types";
import { VertexBuffer } from "@babylonjs/core/Buffers/buffer";
import { generateSoloNavMesh, generateTileCache } from "recast-navigation/generators";
import { Crowd, CrowdAgent, exportNavMesh, getRandomSeed, importNavMesh, NavMesh, NavMeshQuery, setRandomSeed, TileCache } from "recast-navigation";

// Adding Nullable
//   export interface INavigationEnginePluginNullables {
//     addCylinderObstacle (position: Vector3, radius: number, height: number): Nullable<IObstacle>;
//     addBoxObstacle (position: Vector3, extent: Vector3, angle: number): Nullable<IObstacle>;
// }
// // declare module "@babylonjs/core/Navigation/INavigationEngine" {
//   export interface INavigationEnginePlugin {
//     addCylinderObstacle (position: Vector3, radius: number, height: number): Nullable<IObstacle>;
//     addBoxObstacle (position: Vector3, extent: Vector3, angle: number): Nullable<IObstacle>;
//   }
// }

// interface INavigationEnginePluginNullables extends INavigationEnginePlugin  {
//   addCylinderObstacle (position: Vector3, radius: number, height: number): Nullable<IObstacle>;
//   addBoxObstacle (position: Vector3, extent: Vector3, angle: number): Nullable<IObstacle>;
// }

/**
 * RecastJS navigation plugin
 */
export class RecastNavigationJSPlugin implements INavigationEnginePlugin {
  /**
   * Reference to the Recast library
   */
  public bjsRECAST: any = {};

  /**
   * plugin name
   */
  public name: string = "RecastNavigationJSPlugin";

  /**
   * the first navmesh created. We might extend this to support multiple navmeshes
   */
  public navMesh!: NavMesh;
  private _navMeshQuery!: NavMeshQuery;

  private _maximumSubStepCount: number = 10;
  private _timeStep: number = 1 / 60;
  private _timeFactor: number = 1;

  private _tempVec1: any;
  private _tempVec2: any;

  private _tileCache?: TileCache;

  private _worker: Nullable<Worker> = null;

  // TODO: Nullable?
  private _positions: Float32Array = new Float32Array();
  private _indices: Float32Array = new Float32Array();

  /**
   * Initializes the recastJS plugin
   * @param recastInjection can be used to inject your own recast reference
   */
  public constructor() {

    if (!this.isSupported()) {
      Logger.Error("RecastJS is not available. Please make sure you included the js file.");
      return;
    }
    this.setTimeStep();

    this._tempVec1 = { x: 0, y: 0, z: 0 };
    this._tempVec2 = { x: 0, y: 0, z: 0 };
  }

  /**
   * Set worker URL to be used when generating a new navmesh
   * @param workerURL url string
   * @returns boolean indicating if worker is created
   */
  public setWorkerURL (workerURL: string | URL): boolean {
    if (window && window.Worker) {
      this._worker = new Worker(workerURL, {
        type: "module"
      });
      return true;
    }
    return false;
  }

  public setWorker (worker: Worker): boolean {
    if (window && window.Worker) {
      this._worker = worker;
      return true;
    }
    return false;
  }

  /**
   * Set the time step of the navigation tick update.
   * Default is 1/60.
   * A value of 0 will disable fixed time update
   * @param newTimeStep the new timestep to apply to this world.
   */
  setTimeStep (newTimeStep: number = 1 / 60): void {
    this._timeStep = newTimeStep;
  }

  /**
   * Get the time step of the navigation tick update.
   * @returns the current time step
   */
  getTimeStep (): number {
    return this._timeStep;
  }

  /**
   * If delta time in navigation tick update is greater than the time step
   * a number of sub iterations are done. If more iterations are need to reach deltatime
   * they will be discarded.
   * A value of 0 will set to no maximum and update will use as many substeps as needed
   * @param newStepCount the maximum number of iterations
   */
  setMaximumSubStepCount (newStepCount: number = 10): void {
    this._maximumSubStepCount = newStepCount;
  }

  /**
   * Get the maximum number of iterations per navigation tick update
   * @returns the maximum number of iterations
   */
  getMaximumSubStepCount (): number {
    return this._maximumSubStepCount;
  }

  /**
   * Time factor applied when updating crowd agents (default 1). A value of 0 will pause crowd updates.
   * @param value the time factor applied at update
   */
  public set timeFactor (value: number) {
    this._timeFactor = Math.max(value, 0);
  }

  /**
   * Get the time factor used for crowd agent update
   * @returns the time factor
   */
  public get timeFactor (): number {
    return this._timeFactor;
  }

  /**
   * Creates a navigation mesh
   * @param meshes array of all the geometry used to compute the navigation mesh
   * @param parameters bunch of parameters used to filter geometry
   * @param completion callback when data is available from the worker. Not used without a worker
   */
  createNavMesh (meshes: Array<Mesh>, parameters: INavMeshParameters, completion?: (navmeshData: Uint8Array) => void): void {
    if (this._worker && !completion) {
      Logger.Warn("A worker is avaible but no completion callback. Defaulting to blocking navmesh creation");
    } else if (!this._worker && completion) {
      Logger.Warn("A completion callback is avaible but no worker. Defaulting to blocking navmesh creation");
    }

    if (meshes.length === 0) {
      throw new Error("At least one mesh is needed to create the nav mesh.")
    }

    let index: number;
    let tri: number;
    let pt: number;



    const positions = Float32Array.from(meshes[0].getVerticesData(VertexBuffer.PositionKind, false, false) ?? []);
    const indices = Float32Array.from(meshes[0].getIndices() ?? [])

    this._positions = positions;

    this._indices = indices;

    // const indices = Float32Array.from(this._indices);
    // const positions = Float32Array.from(this._positions);



    // const meshesToMerge: Mesh[] = [];

    // for (const mesh of meshes) {
    //   const positionAttribute = mesh.geometry.attributes
    //     .position as BufferAttribute;

    //   if (!positionAttribute || positionAttribute.itemSize !== 3) {
    //     continue;
    //   }

    //   let meshToMerge = mesh;
    //   const index: ArrayLike<number> | undefined = mesh.geometry.getIndex()?.array;

    //   if (index === undefined) {
    //     meshToMerge = meshToMerge.clone();
    //     meshToMerge.geometry = mesh.geometry.clone();

    //     // this will become indexed when merging with other meshes
    //     const ascendingIndex: number[] = [];
    //     for (let i = 0; i < positionAttribute.count; i++) {
    //       ascendingIndex.push(i);
    //     }

    //     meshToMerge.geometry.setIndex(ascendingIndex);
    //   }

    //   meshesToMerge.push(meshToMerge);
    // }

    // const mergedPositions: number[] = [];
    // const mergedIndices: number[] = [];

    // const positionToIndex: { [hash: string]: number } = {};
    // let indexCounter = 0;

    // for (const mesh of meshesToMerge) {
    //   mesh.updateMatrixWorld();

    //   const positions = mesh.geometry.attributes.position.array;
    //   const index = mesh.geometry.getIndex()!.array;

    //   for (let i = 0; i < index.length; i++) {
    //     const pt = index[i] * 3;

    //     const pos = tmpVec3.set(
    //       positions[pt],
    //       positions[pt + 1],
    //       positions[pt + 2]
    //     );
    //     mesh.localToWorld(pos);

    //     const key = `${pos.x}_${pos.y}_${pos.z}`;
    //     let idx = positionToIndex[key];

    //     if (!idx) {
    //       positionToIndex[key] = idx = indexCounter;
    //       mergedPositions.push(pos.x, pos.y, pos.z);
    //       indexCounter++;
    //     }

    //     mergedIndices.push(idx);
    //   }
    // }

    // return [Float32Array.from(mergedPositions), Uint32Array.from(mergedIndices)];


    // let offset = 0;
    // for (index = 0; index < meshes.length; index++) {
    //   if (meshes[index]) {
    //     const mesh = meshes[index];

    //     const meshIndices = mesh.getIndices();
    //     if (!meshIndices) {
    //       continue;
    //     }
    //     const meshPositions = mesh.getVerticesData(VertexBuffer.PositionKind, false, false);
    //     if (!meshPositions) {
    //       continue;
    //     }

    //     const worldMatrices = [];
    //     const worldMatrix = mesh.computeWorldMatrix(true);

    //     if (mesh.hasThinInstances) {
    //       const thinMatrices = (mesh as Mesh).thinInstanceGetWorldMatrices();
    //       for (let instanceIndex = 0; instanceIndex < thinMatrices.length; instanceIndex++) {
    //         const tmpMatrix = new Matrix();
    //         const thinMatrix = thinMatrices[instanceIndex];
    //         thinMatrix.multiplyToRef(worldMatrix, tmpMatrix);
    //         worldMatrices.push(tmpMatrix);
    //       }
    //     } else {
    //       worldMatrices.push(worldMatrix);
    //     }

    //     for (let matrixIndex = 0; matrixIndex < worldMatrices.length; matrixIndex++) {
    //       const wm = worldMatrices[matrixIndex];
    //       for (tri = 0; tri < meshIndices.length; tri++) {
    //         indices.push(meshIndices[tri] + offset);
    //       }

    //       const transformed = Vector3.Zero();
    //       const position = Vector3.Zero();
    //       for (pt = 0; pt < meshPositions.length; pt += 3) {
    //         Vector3.FromArrayToRef(meshPositions, pt, position);
    //         Vector3.TransformCoordinatesToRef(position, wm, transformed);
    //         positions.push(transformed.x, transformed.y, transformed.z);
    //       }

    //       offset += meshPositions.length / 3;
    //     }
    //   }
    // }


    // TODO:
    // const rc = new this.bjsRECAST.rcConfig();
    // rc.cs = parameters.cs;
    // rc.ch = parameters.ch;
    // rc.borderSize = parameters.borderSize ? parameters.borderSize : 0;
    // rc.tileSize = parameters.tileSize ? parameters.tileSize : 0;
    // rc.walkableSlopeAngle = parameters.walkableSlopeAngle;
    // rc.walkableHeight = parameters.walkableHeight;
    // rc.walkableClimb = parameters.walkableClimb;
    // rc.walkableRadius = parameters.walkableRadius;
    // rc.maxEdgeLen = parameters.maxEdgeLen;
    // rc.maxSimplificationError = parameters.maxSimplificationError;
    // rc.minRegionArea = parameters.minRegionArea;
    // rc.mergeRegionArea = parameters.mergeRegionArea;
    // rc.maxVertsPerPoly = parameters.maxVertsPerPoly;
    // rc.detailSampleDist = parameters.detailSampleDist;
    // rc.detailSampleMaxError = parameters.detailSampleMaxError;
    if (this._worker && completion) {
      // spawn worker and send message
      const config = {
        cs: 0.05,
        ch: 0.2,
      };
      this._worker.postMessage({ positions, indices, config }, [
        positions.buffer,
        indices.buffer,
      ]);
      this._worker.onmessage = (e) => {
        this.buildFromNavmeshData(e.data);
        completion(e.data);

        this._navMeshQuery = new NavMeshQuery(this.navMesh)
      };
    } else {
      // blocking calls



      if (!positions || !indices) {
        throw new Error("Unable to get nav mesh. No vertices or indices.");

      }

      const { success, navMesh } = generateSoloNavMesh(positions, indices);
      if (!success) {
        throw new Error("Unable to generateSoloNavMesh");
      }

      this.navMesh = navMesh;

      this._navMeshQuery = new NavMeshQuery(navMesh)
    }
  }

  /**
   * Create a navigation mesh debug mesh
   * @param scene is where the mesh will be added
   * @returns debug display mesh
   */
  createDebugNavMesh (scene: Scene): Mesh {
    // let tri: number;
    // let pt: number;
    const [positions, indices] = this.navMesh.getDebugNavMesh();

    // for (tri = 0; tri < triangleCount * 3; tri++) {
    //   indices.push(tri);
    // }
    // for (tri = 0; tri < triangleCount; tri++) {
    //   for (pt = 0; pt < 3; pt++) {
    //     const point = debugNavMesh.getTriangle(tri).getPoint(pt);
    //     positions.push(point.x, point.y, point.z);
    //   }
    // }

    const mesh = new Mesh("NavMeshDebug", scene);
    const vertexData = new VertexData();

    vertexData.indices = indices;
    vertexData.positions = positions;
    vertexData.applyToMesh(mesh, false);
    return mesh;
  }

  /**
   * Get a navigation mesh constrained position, closest to the parameter position
   * @param position world position
   * @returns the closest point to position constrained by the navigation mesh
   */
  getClosestPoint (position: Vector3): Vector3 {
    this._tempVec1.x = position.x;
    this._tempVec1.y = position.y;
    this._tempVec1.z = position.z;
    const ret = this._navMeshQuery.findClosestPoint(this._tempVec1);
    const pr = new Vector3(ret.point.x, ret.point.y, ret.point.z);
    return pr;
  }

  /**
   * Get a navigation mesh constrained position, closest to the parameter position
   * @param position world position
   * @param result output the closest point to position constrained by the navigation mesh
   */
  getClosestPointToRef (position: Vector3, result: Vector3): void {
    this._tempVec1.x = position.x;
    this._tempVec1.y = position.y;
    this._tempVec1.z = position.z;
    const ret = this._navMeshQuery.findClosestPoint(this._tempVec1);
    result.set(ret.point.x, ret.point.y, ret.point.z);
  }

  /**
   * Get a navigation mesh constrained position, within a particular radius
   * @param position world position
   * @param maxRadius the maximum distance to the constrained world position
   * @returns the closest point to position constrained by the navigation mesh
   */
  getRandomPointAround (position: Vector3, maxRadius: number): Vector3 {
    this._tempVec1.x = position.x;
    this._tempVec1.y = position.y;
    this._tempVec1.z = position.z;
    const ret = this._navMeshQuery.findRandomPointAroundCircle(this._tempVec1, maxRadius);
    const pr = new Vector3(ret.randomPoint.x, ret.randomPoint.y, ret.randomPoint.z);
    return pr;
  }

  /**
   * Get a navigation mesh constrained position, within a particular radius
   * @param position world position
   * @param maxRadius the maximum distance to the constrained world position
   * @param result output the closest point to position constrained by the navigation mesh
   */
  getRandomPointAroundToRef (position: Vector3, maxRadius: number, result: Vector3): void {
    this._tempVec1.x = position.x;
    this._tempVec1.y = position.y;
    this._tempVec1.z = position.z;
    const ret = this._navMeshQuery.findRandomPointAroundCircle(this._tempVec1, maxRadius);
    result.set(ret.randomPoint.x, ret.randomPoint.y, ret.randomPoint.z);
  }

  /**
   * Compute the final position from a segment made of destination-position
   * @param position world position
   * @param destination world position
   * @returns the resulting point along the navmesh
   */
  moveAlong (position: Vector3, destination: Vector3): Vector3 {
    this._tempVec1.x = position.x;
    this._tempVec1.y = position.y;
    this._tempVec1.z = position.z;
    this._tempVec2.x = destination.x;
    this._tempVec2.y = destination.y;
    this._tempVec2.z = destination.z;
    const ret = this._navMeshQuery.moveAlongSurface(0, this._tempVec1, this._tempVec2);
    const pr = new Vector3(ret.resultPosition.x, ret.resultPosition.y, ret.resultPosition.z);
    return pr;
  }

  /**
   * Compute the final position from a segment made of destination-position
   * @param position world position
   * @param destination world position
   * @param result output the resulting point along the navmesh
   */
  moveAlongToRef (position: Vector3, destination: Vector3, result: Vector3): void {
    this._tempVec1.x = position.x;
    this._tempVec1.y = position.y;
    this._tempVec1.z = position.z;
    this._tempVec2.x = destination.x;
    this._tempVec2.y = destination.y;
    this._tempVec2.z = destination.z;
    const ret = this._navMeshQuery.moveAlongSurface(0, this._tempVec1, this._tempVec2);
    result.set(ret.resultPosition.x, ret.resultPosition.y, ret.resultPosition.z);
  }

  private _convertNavPathPoints (navPath: any): Vector3[] {
    let pt: number;
    const pointCount = navPath.getPointCount();
    const positions = [];
    for (pt = 0; pt < pointCount; pt++) {
      const p = navPath.getPoint(pt);
      positions.push(new Vector3(p.x, p.y, p.z));
    }
    return positions;
  }

  /**
   * Compute a navigation path from start to end. Returns an empty array if no path can be computed
   * Path is straight.
   * @param start world position
   * @param end world position
   * @returns array containing world position composing the path
   */
  computePath (start: Vector3, end: Vector3): Vector3[] {
    this._tempVec1.x = start.x;
    this._tempVec1.y = start.y;
    this._tempVec1.z = start.z;
    this._tempVec2.x = end.x;
    this._tempVec2.y = end.y;
    this._tempVec2.z = end.z;
    const navPath = this._navMeshQuery.computePath(this._tempVec1, this._tempVec2);
    return this._convertNavPathPoints(navPath);
  }

  /**
   * Compute a navigation path from start to end. Returns an empty array if no path can be computed.
   * Path follows navigation mesh geometry.
   * @param start world position
   * @param end world position
   * @returns array containing world position composing the path
   */
  computePathSmooth (start: Vector3, end: Vector3): Vector3[] {
    this._tempVec1.x = start.x;
    this._tempVec1.y = start.y;
    this._tempVec1.z = start.z;
    this._tempVec2.x = end.x;
    this._tempVec2.y = end.y;
    this._tempVec2.z = end.z;
    // TODO: computePathSmooth
    const navPath = this._navMeshQuery.computePath(this._tempVec1, this._tempVec2);
    return this._convertNavPathPoints(navPath);
  }
  /**
   * Create a new Crowd so you can add agents
   * @param maxAgents the maximum agent count in the crowd
   * @param maxAgentRadius the maximum radius an agent can have
   * @param scene to attach the crowd to
   * @returns the crowd you can add agents to
   */
  createCrowd (maxAgents: number, maxAgentRadius: number, scene: Scene): ICrowd {
    const crowd = new RecastJSCrowd(this, maxAgents, maxAgentRadius, scene);
    return crowd;
  }

  /**
   * Set the Bounding box extent for doing spatial queries (getClosestPoint, getRandomPointAround, ...)
   * The queries will try to find a solution within those bounds
   * default is (1,1,1)
   * @param extent x,y,z value that define the extent around the queries point of reference
   */
  setDefaultQueryExtent (extent: Vector3): void {
    this._tempVec1.x = extent.x;
    this._tempVec1.y = extent.y;
    this._tempVec1.z = extent.z;
    this._navMeshQuery.defaultQueryHalfExtents = this._tempVec1;
  }

  /**
   * Get the Bounding box extent specified by setDefaultQueryExtent
   * @returns the box extent values
   */
  getDefaultQueryExtent (): Vector3 {
    this._tempVec1 = this._navMeshQuery.defaultQueryHalfExtents;
    return new Vector3(this._tempVec1.x, this._tempVec1.y, this._tempVec1.z);
  }

  /**
 * Get the Bounding box extent result specified by setDefaultQueryExtent
 * @param result output the box extent values
 */
  getDefaultQueryExtentToRef (result: Vector3): void {
    this._tempVec1 = this._navMeshQuery.defaultQueryHalfExtents;
    result.set(this._tempVec1.x, this._tempVec1.y, this._tempVec1.z);
  }


  /**
   * build the navmesh from a previously saved state using getNavmeshData
   * @param data the Uint8Array returned by getNavmeshData
   */
  buildFromNavmeshData (data: Uint8Array): void {
    const result = importNavMesh(data);
    this.navMesh = result.navMesh;
  }

  /**
   * returns the navmesh data that can be used later. The navmesh must be built before retrieving the data
   * @returns data the Uint8Array that can be saved and reused
   */
  getNavmeshData (): Uint8Array {
    return exportNavMesh(this.navMesh)
  }

  /**
   * Disposes
   */
  public dispose () { }

  private _createTileCache () {
    if (!this._tileCache) {
      const { success, navMesh, tileCache } = generateTileCache(this._positions, this._indices, {
        /* ... */
        tileSize: 16,
      });
      if (!success) {
        console.error("Unable to addCylinderObstacle.");
        return null;
      }
      this._tileCache = tileCache;
      this.navMesh = navMesh;
    }
  }

  /**
   * Creates a cylinder obstacle and add it to the navigation
   * @param position world position
   * @param radius cylinder radius
   * @param height cylinder height
   * @returns the obstacle freshly created
   */
  addCylinderObstacle (position: Vector3, radius: number, height: number): IObstacle {
    this._createTileCache();

    this._tempVec1.x = position.x;
    this._tempVec1.y = position.y;
    this._tempVec1.z = position.z;
    return this._tileCache?.addCylinderObstacle(this._tempVec1, radius, height) ?? null as any as IObstacle;
  }

  /**
   * Creates an oriented box obstacle and add it to the navigation
   * @param position world position
   * @param extent box size
   * @param angle angle in radians of the box orientation on Y axis
   * @returns the obstacle freshly created
   */
  addBoxObstacle (position: Vector3, extent: Vector3, angle: number): IObstacle {
    this._createTileCache();

    this._tempVec1.x = position.x;
    this._tempVec1.y = position.y;
    this._tempVec1.z = position.z;
    this._tempVec2.x = extent.x;
    this._tempVec2.y = extent.y;
    this._tempVec2.z = extent.z;
    return this._tileCache?.addBoxObstacle(this._tempVec1, this._tempVec2, angle) ?? null as any as IObstacle;
  }

  /**
   * Removes an obstacle created by addCylinderObstacle or addBoxObstacle
   * @param obstacle obstacle to remove from the navigation
   */
  removeObstacle (obstacle: IObstacle): void {
    this._tileCache?.removeObstacle(obstacle);
  }

  /**
   * If this plugin is supported
   * @returns true if plugin is supported
   */
  public isSupported (): boolean {
    return true;
  }

  /**
   * Returns the seed used for randomized functions like `getRandomPointAround`
   * @returns seed number
   */
  public getRandomSeed (): number {
    return getRandomSeed();
  }

  /**
   * Set the seed used for randomized functions like `getRandomPointAround`
   * @param seed number used as seed for random functions
   */
  public setRandomSeed (seed: number): void {
    setRandomSeed(seed);
  }
}

/**
 * Recast detour crowd implementation
 */
export class RecastJSCrowd implements ICrowd {
  /**
   * Recast/detour plugin
   */
  public bjsRECASTPlugin: RecastNavigationJSPlugin;
  /**
   * Link to the detour crowd
   */
  public recastCrowd: Crowd;
  /**
   * One transform per agent
   */
  public transforms: TransformNode[] = new Array<TransformNode>();
  /**
   * All agents created
   */
  public agents: number[] = new Array<number>();
  /**
   * agents reach radius
   */
  public reachRadii: number[] = new Array<number>();
  /**
   * true when a destination is active for an agent and notifier hasn't been notified of reach
   */
  private _agentDestinationArmed: boolean[] = new Array<boolean>();
  /**
   * agent current target
   */
  private _agentDestination: Vector3[] = new Array<Vector3>();
  /**
   * Link to the scene is kept to unregister the crowd from the scene
   */
  private _scene: Scene;

  /**
   * Observer for crowd updates
   */
  private _onBeforeAnimationsObserver: Nullable<Observer<Scene>> = null;

  /**
   * Fires each time an agent is in reach radius of its destination
   */
  public onReachTargetObservable = new Observable<{ agentIndex: number; destination: Vector3 }>();

  /**
   * Constructor
   * @param plugin recastJS plugin
   * @param maxAgents the maximum agent count in the crowd
   * @param maxAgentRadius the maximum radius an agent can have
   * @param scene to attach the crowd to
   * @returns the crowd you can add agents to
   */
  public constructor(plugin: RecastNavigationJSPlugin, maxAgents: number, maxAgentRadius: number, scene: Scene) {
    this.bjsRECASTPlugin = plugin;
    this.recastCrowd = new Crowd(plugin.navMesh, {
      maxAgents,
      maxAgentRadius,
    })
    this._scene = scene;

    this._onBeforeAnimationsObserver = scene.onBeforeAnimationsObservable.add(() => {
      this.update(scene.getEngine().getDeltaTime() * 0.001 * plugin.timeFactor);
    });
  }

  /**
   * Add a new agent to the crowd with the specified parameter a corresponding transformNode.
   * You can attach anything to that node. The node position is updated in the scene update tick.
   * @param pos world position that will be constrained by the navigation mesh
   * @param parameters agent parameters
   * @param transform hooked to the agent that will be update by the scene
   * @returns agent index
   */
  addAgent (pos: Vector3, parameters: IAgentParameters, transform: TransformNode): number {
    const agentParams: IAgentParameters = {

      radius: parameters.radius,
      height: parameters.height,
      maxAcceleration: parameters.maxAcceleration,
      maxSpeed: parameters.maxSpeed,
      collisionQueryRange: parameters.collisionQueryRange,
      pathOptimizationRange: parameters.pathOptimizationRange,
      separationWeight: parameters.separationWeight,
      reachRadius: parameters.reachRadius ? parameters.reachRadius : parameters.radius
      // reachRadius: 0
      // updateFlags : 7,
      // obstacleAvoidanceType : 0,
      // queryFilterType : 0,
      // userData : 0,
    }

    const agent = this.recastCrowd.addAgent({ x: pos.x, y: pos.y, z: pos.z }, agentParams);
    this.transforms.push(transform);
    this.agents.push(agent.agentIndex);
    // this.reachRadii.push(parameters.reachRadius ? parameters.reachRadius : parameters.radius);
    this._agentDestinationArmed.push(false);
    this._agentDestination.push(new Vector3(0, 0, 0));
    return agent.agentIndex;
  }

  /**
   * Returns the agent position in world space
   * @param index agent index returned by addAgent
   * @returns world space position
   */
  getAgentPosition (index: number): Vector3 {
    const agentPos = this.recastCrowd.getAgent(index)?.position() ?? { x: 0, y: 0, z: 0 };
    return new Vector3(agentPos.x, agentPos.y, agentPos.z);
  }

  /**
   * Returns the agent position result in world space
   * @param index agent index returned by addAgent
   * @param result output world space position
   */
  getAgentPositionToRef (index: number, result: Vector3): void {
    const agentPos = this.recastCrowd.getAgent(index)?.position() ?? { x: 0, y: 0, z: 0 };
    result.set(agentPos.x, agentPos.y, agentPos.z);
  }

  /**
   * Returns the agent velocity in world space
   * @param index agent index returned by addAgent
   * @returns world space velocity
   */
  getAgentVelocity (index: number): Vector3 {
    const agentVel = this.recastCrowd.getAgent(index)?.velocity() ?? { x: 0, y: 0, z: 0 };
    return new Vector3(agentVel.x, agentVel.y, agentVel.z);
  }

  /**
   * Returns the agent velocity result in world space
   * @param index agent index returned by addAgent
   * @param result output world space velocity
   */
  getAgentVelocityToRef (index: number, result: Vector3): void {
    const agentVel = this.recastCrowd.getAgent(index)?.velocity() ?? { x: 0, y: 0, z: 0 };
    result.set(agentVel.x, agentVel.y, agentVel.z);
  }

  /**
   * Returns the agent next target point on the path
   * @param index agent index returned by addAgent
   * @returns world space position
   */
  getAgentNextTargetPath (index: number): Vector3 {
    const pathTargetPos = this.recastCrowd.getAgent(index)?.nextTargetInPath() ?? { x: 0, y: 0, z: 0 };
    return new Vector3(pathTargetPos.x, pathTargetPos.y, pathTargetPos.z);
  }

  /**
   * Returns the agent next target point on the path
   * @param index agent index returned by addAgent
   * @param result output world space position
   */
  getAgentNextTargetPathToRef (index: number, result: Vector3): void {
    const pathTargetPos = this.recastCrowd.getAgent(index)?.nextTargetInPath() ?? { x: 0, y: 0, z: 0 };
    result.set(pathTargetPos.x, pathTargetPos.y, pathTargetPos.z);
  }

  /**
   * Gets the agent state
   * @param index agent index returned by addAgent
   * @returns agent state
   */
  getAgentState (index: number): number {
    return this.recastCrowd.getAgent(index)?.state() ?? 0; // invalid
  }

  /**
   * returns true if the agent in over an off mesh link connection
   * @param index agent index returned by addAgent
   * @returns true if over an off mesh link connection
   */
  overOffmeshConnection (index: number): boolean {
    return this.recastCrowd.getAgent(index)?.overOffMeshConnection() ?? false;
  }

  /**
   * Asks a particular agent to go to a destination. That destination is constrained by the navigation mesh
   * @param index agent index returned by addAgent
   * @param destination targeted world position
   */
  agentGoto (index: number, destination: Vector3): void {
    this.recastCrowd.getAgent(index)?.requestMoveTarget(destination);

    // arm observer
    const item = this.agents.indexOf(index);
    if (item > -1) {
      this._agentDestinationArmed[item] = true;
      this._agentDestination[item].set(destination.x, destination.y, destination.z);
    }
  }

  /**
   * Teleport the agent to a new position
   * @param index agent index returned by addAgent
   * @param destination targeted world position
   */
  agentTeleport (index: number, destination: Vector3): void {
    this.recastCrowd.getAgent(index)?.teleport(destination);
  }

  /**
   * Update agent parameters
   * @param index agent index returned by addAgent
   * @param parameters agent parameters
   */
  updateAgentParameters (index: number, parameters: IAgentParameters): void {
    const agent = this.recastCrowd.getAgent(index)
    if (!agent) {
      return
    }

    const agentParams = agent.parameters();

    if (!agentParams) {
      return
    }

    if (parameters.radius !== undefined) {
      agentParams.radius = parameters.radius;
    }
    if (parameters.height !== undefined) {
      agentParams.height = parameters.height;
    }
    if (parameters.maxAcceleration !== undefined) {
      agentParams.maxAcceleration = parameters.maxAcceleration;
    }
    if (parameters.maxSpeed !== undefined) {
      agentParams.maxSpeed = parameters.maxSpeed;
    }
    if (parameters.collisionQueryRange !== undefined) {
      agentParams.collisionQueryRange = parameters.collisionQueryRange;
    }
    if (parameters.pathOptimizationRange !== undefined) {
      agentParams.pathOptimizationRange = parameters.pathOptimizationRange;
    }
    if (parameters.separationWeight !== undefined) {
      agentParams.separationWeight = parameters.separationWeight;
    }

    agent.updateParameters(agentParams);
  }

  /**
   * remove a particular agent previously created
   * @param index agent index returned by addAgent
   */
  removeAgent (index: number): void {
    this.recastCrowd.removeAgent(index);

    const item = this.agents.indexOf(index);
    if (item > -1) {
      this.agents.splice(item, 1);
      this.transforms.splice(item, 1);
      this.reachRadii.splice(item, 1);
      this._agentDestinationArmed.splice(item, 1);
      this._agentDestination.splice(item, 1);
    }
  }

  /**
   * get the list of all agents attached to this crowd
   * @returns list of agent indices
   */
  getAgents (): number[] {
    return this.agents;
  }

  /**
   * Tick update done by the Scene. Agent position/velocity/acceleration is updated by this function
   * @param deltaTime in seconds
   */
  update (deltaTime: number): void {
    // update obstacles
    this.recastCrowd.update(deltaTime);

    if (deltaTime <= Epsilon) {
      return;
    }
    // update crowd
    const timeStep = this.bjsRECASTPlugin.getTimeStep();
    const maxStepCount = this.bjsRECASTPlugin.getMaximumSubStepCount();
    if (timeStep <= Epsilon) {
      this.recastCrowd.update(deltaTime);
    } else {
      let iterationCount = Math.floor(deltaTime / timeStep);
      if (maxStepCount && iterationCount > maxStepCount) {
        iterationCount = maxStepCount;
      }
      if (iterationCount < 1) {
        iterationCount = 1;
      }

      const step = deltaTime / iterationCount;
      for (let i = 0; i < iterationCount; i++) {
        this.recastCrowd.update(step);
      }
    }

    // update transforms
    for (let index = 0; index < this.agents.length; index++) {
      // update transform position
      const agentIndex = this.agents[index];
      const agentPosition = this.getAgentPosition(agentIndex);
      this.transforms[index].position = agentPosition;
      // check agent reach destination
      if (this._agentDestinationArmed[index]) {
        const dx = agentPosition.x - this._agentDestination[index].x;
        const dz = agentPosition.z - this._agentDestination[index].z;
        const radius = this.reachRadii[index];
        const groundY = this._agentDestination[index].y - this.reachRadii[index];
        const ceilingY = this._agentDestination[index].y + this.reachRadii[index];
        const distanceXZSquared = dx * dx + dz * dz;
        if (agentPosition.y > groundY && agentPosition.y < ceilingY && distanceXZSquared < radius * radius) {
          this._agentDestinationArmed[index] = false;
          this.onReachTargetObservable.notifyObservers({ agentIndex: agentIndex, destination: this._agentDestination[index] });
        }
      }
    }
  }

  /**
   * Set the Bounding box extent for doing spatial queries (getClosestPoint, getRandomPointAround, ...)
   * The queries will try to find a solution within those bounds
   * default is (1,1,1)
   * @param extent x,y,z value that define the extent around the queries point of reference
   */
  setDefaultQueryExtent (extent: Vector3): void {
    const ext = new this.bjsRECASTPlugin.bjsRECAST.Vec3(extent.x, extent.y, extent.z);
    this.bjsRECASTPlugin.setDefaultQueryExtent(ext);
  }

  /**
   * Get the Bounding box extent specified by setDefaultQueryExtent
   * @returns the box extent values
   */
  getDefaultQueryExtent (): Vector3 {
    const p = this.bjsRECASTPlugin.getDefaultQueryExtent();
    return new Vector3(p.x, p.y, p.z);
  }

  /**
   * Get the Bounding box extent result specified by setDefaultQueryExtent
   * @param result output the box extent values
   */
  getDefaultQueryExtentToRef (result: Vector3): void {
    const p = this.bjsRECASTPlugin.getDefaultQueryExtent();
    result.set(p.x, p.y, p.z);
  }

  /**
   * Get the next corner points composing the path (max 4 points)
   * @param index agent index returned by addAgent
   * @returns array containing world position composing the path
   */
  getCorners (index: number): Vector3[] {
    // const straightPath = this.bjsRECASTPlugin.findStraightPath(startPos, endPos, path, maxPathLength);

    // // The straightPath will contain the corners of the path
    // for (let i = 0; i < straightPath.length; i++) {
    //   const corner = straightPath[i];
    //   console.log(`Corner ${i}:`, corner);
    // }

    // let pt: number;
    // const navPath = this.recastCrowd.getCorners(index);
    // const pointCount = navPath.getPointCount();
    // const positions = [];
    // for (pt = 0; pt < pointCount; pt++) {
    //   const p = navPath.getPoint(pt);
    //   positions.push(new Vector3(p.x, p.y, p.z));
    // }
    // return positions;
    return []
  }

  /**
   * Release all resources
   */
  dispose (): void {
    this.recastCrowd.destroy();
    this._scene.onBeforeAnimationsObservable.remove(this._onBeforeAnimationsObserver);
    this._onBeforeAnimationsObserver = null;
    this.onReachTargetObservable.clear();
  }
}
