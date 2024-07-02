using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class MassSpringCloth : MonoBehaviour
{

    public enum Integration
    {
        EXPLICIT = 0,
        SYMPLECTIC = 1,
    };

    //Stores edges as a struct where vertex_a is always smaller than vertex_b. This is done so that we can remove duplicates later on.
    struct Edge
    {
        public int vertex_a;
        public int vertex_b;
        public Edge(int a, int b)
        {
            if (a <= b)
            {
                this.vertex_a = a;
                this.vertex_b = b;
            }
            else
            {
                this.vertex_a = b;
                this.vertex_b = a;
            }
        }
    };

    #region InEditorVariables
    //Physics management public variables:
    public bool paused;
    public float time_step;
    public Vector3 gravity;
    public Integration integration_method;
    public int substeps; //determines the number of substeps to be performed for each FixedStep call.

    //Spring Cloth public variables:
    public float node_mass; //contains a default value for the mass of each node.
    public bool update_node_mass;
    public float spring_stiffness_traccion; //contains a default value for the stiffness of each spring.
    public float spring_stiffness_flexion;
    public bool update_spring_stiffness;
    public Collider[] fixers; //contains a list of the fixers that will be used to fix nodes based on the mesh's vertices that they collide with.
    public SphereCollider[] colliders_penalty_forces, colliders_pull_forces; //contains a list of the collider objects.

    //damping variables:
    public float damping_node;
    public float damping_spring;

    //wind variables
    public Vector3 wind_vector; //this vector should always be normalized, so we'll normalize it on the Init() function.
    public float wind_force;

    //penalty force variables
    public float penalty_force; //force applied due to penalty when colliding with sphere colliders.
    public float penalty_penetration_distance_margin; //margin of error of radius that must be added to smooth out the calculations.

    //pull force variables
    public float pull_force;
    public float pull_penetration_distance_margin;

    //other public variables:
    public bool debug_draw; //determines if it should perform debug drawing during runtime.
    #endregion

    #region OtherVariables
    //Nodes and Springs:
    private List<Node> nodes;
    private List<Spring> springs_traccion;
    private List<Spring> springs_flexion;

    //List of nodes fixed by each fixer:
    class FixerInfo //make it a class instead of a struct, because structs are passed by value in C# and classes by reference / pointer, and this is what we want in this case, cause we NEED to be able to modify these in the update function.
    {
        public Vector3 old_position; //the last position of this fixer.
        public List<Node> fixed_nodes; //list of pointers to the nodes that were fixed by this fixer.

        public FixerInfo(Vector3 starting_position)
        {
            this.old_position = starting_position;
            this.fixed_nodes = new List<Node>();
        }
    };
    private Dictionary<Collider, FixerInfo> fixers_info; //contains information about the colliders used for fixers and the fixed nodes. Whenever a fixer is created, we store its old position. Whenever a fixer is moved, if its position no longer matches the old stored position, we will iterate over all the children nodes and move them over by the distance moved, and then we will update the old position variable to contain the new one.


    //Spring Cloth private variables:
    private Mesh mesh; //contains a ptr to the parent's mesh
    private Vector3[] vertices; //contains a list of its vertices
    private int[] triangles; //contains a list of its triangles
    //private List<Edge> edges; //contains a list of all the added edges. If any edge is a duplicate, it will not be added to prevent double edges from appearing.
    private Dictionary<Edge, int> edges; //this also contains a list of all the added edges, as well as the opposite vertex that each edge had when it was added. This way, we can both prevent duplicate edges from appearing, as well as constructing springs across opposite triangles to simulate more complex forces.
    #endregion

    #region MonoBehaviour
    public MassSpringCloth()
    {
        //we cannot call this on the constructor because all of this logic depends on behaviour that an Unity monobehaviour cannot invoke on its constructor!
        ////We can call it instead on Start(), for example...
        //this.Init();
    }

    public void Init()
    {
        this.nodes = new List<Node>();
        this.springs_traccion = new List<Spring>();
        this.springs_flexion = new List<Spring>();
        this.edges = new Dictionary<Edge, int>();
        this.fixers_info = new Dictionary<Collider, FixerInfo>();
        this.wind_vector.Normalize(); //normalize just in case someone inputs a non-normalized wind vector!
        this.CreateMeshData();
        this.CreateNodes();
        this.CreateEdgesAndSprings();
    }

    // Start is called before the first frame update
    void Start()
    {
        this.Init();
    }

    // Update is called once per frame
    void Update()
    {
        //print("Update");
        if (Input.GetKeyUp(KeyCode.P))
            this.paused = !this.paused;
        
        this.UpdateVertexPositions();
        this.UpdateSurfaceNormals();
        this.UpdateFixedNodePositions();

        if (this.debug_draw)
            this.DebugDraw();


        //This is a good old hacky trick I learned back in the day (TM) when working with Unreal Engine 4's skysphere BP.
        //Basically, the only reason this exists is to simulate some kind of "event" to update certain values from the editor's user interface.
        //In this case, it prevents exploding the simulation when typing a value (for example, if you remove the number from the input box, it would be interpreted as 0,
        //which would break the simulation even tho you actually wanted to type a different value...).
        if (this.update_node_mass)
        {
            this.update_node_mass = false;
            foreach (var node in this.nodes)
            {
                node.mass = this.node_mass;
            }
        }

        //same thing here, good old hacky bool trick.
        if (this.update_spring_stiffness)
        {
            this.update_spring_stiffness = false;
            foreach (var spring in this.springs_traccion)
            {
                spring.stiffness = this.spring_stiffness_traccion;
            }
            foreach (var spring in this.springs_flexion)
            {
                spring.stiffness = this.spring_stiffness_flexion;
            }
        }

        //WARNING: Do note that updating spring stiffness during runtime without pausing the simulation is a "dangerous" process.
        //It is doable, but it should be adjusted slowly, otherwise, you risk exploding the simulation's integration.
        //It is best to set the stiffness to some value on startup, but the option to manually tweak it during runtime is there in case you want it.
        //The best way to update parameters that would cause the simulation to explode between steps is to pause the simulation and update the values to prevent weird integration steps from blowing the cloth away.

    }

    void FixedUpdate()
    {
        //print("FixedUpdate");
        if (this.paused)
            return;

        this.SubStep(this.substeps);
    }

    #endregion

    #region StepFunctions
    private void SubStep(int num)
    {
        for (int i = 0; i < num; ++i)
        {
            this.Step();
        }
    }
    private void Step()
    {
        switch (this.integration_method)
        {
            case Integration.EXPLICIT:
                this.stepExplicit();
                break;
            case Integration.SYMPLECTIC:
                this.stepSymplectic();
                break;
            default:
                throw new System.Exception("ERROR : THIS SHOULD NEVER HAPPEN, ELLIS, WTF DID YOU JUST DO????");
        }
    }

    void SimulateForces()
    {
        this.SimulateWindForces();
        this.SimulatePenaltyForces();
        this.SimulatePullForces();
    }

    private void stepExplicit()
    {
        foreach (var node in this.nodes)
        {
            node.force = Vector3.zero;
            node.acceleration = this.gravity;
            node.ComputeForce(this.damping_node);
        }
        this.SimulateForces();

        foreach (var spring in this.springs_traccion)
        {
            spring.ComputeForce(this.damping_spring);
        }
        foreach (var spring in this.springs_flexion)
        {
            spring.ComputeForce(this.damping_spring);
        }

        foreach (var node in this.nodes)
        {
            if (!node.is_fixed)
            {
                node.position += time_step * node.velocity;
                node.velocity += this.time_step / node.mass * node.force;
            }
        }

        foreach (var spring in this.springs_traccion)
        {
            spring.UpdateLength();
        }
        foreach (var spring in this.springs_flexion)
        {
            spring.UpdateLength();
        }
    }

    private void stepSymplectic()
    {
        foreach (var node in this.nodes)
        {
            node.force = Vector3.zero;
            node.acceleration = this.gravity;
            node.ComputeForce(this.damping_node);
        }
        this.SimulateForces();

        foreach (var spring in this.springs_traccion)
        {
            spring.ComputeForce(this.damping_spring);
        }
        foreach (var spring in this.springs_flexion)
        {
            spring.ComputeForce(this.damping_spring);
        }

        foreach (var node in this.nodes)
        {
            if (!node.is_fixed)
            {
                node.velocity += this.time_step / node.mass * node.force;
                node.position += time_step * node.velocity;
            }
        }

        foreach (var spring in this.springs_traccion)
        {
            spring.UpdateLength();
        }
        foreach (var spring in this.springs_flexion)
        {
            spring.UpdateLength();
        }

    }

    #endregion

    #region OtherMethods
    private void UpdateFixedNodePositions()
    {
        foreach (var fixer_info_pair in this.fixers_info)
        {
            Collider collider = fixer_info_pair.Key;
            FixerInfo info = fixer_info_pair.Value;
            if (collider.transform.position != info.old_position)
            {
                Vector3 movement_vector = collider.transform.position - info.old_position;
                foreach (var node in info.fixed_nodes)
                {
                    node.position = node.position + movement_vector;
                }
                info.old_position = collider.transform.position;
            }
        }
    }

    #endregion

    #region MassSpringMeshMethods
    private void CreateMeshData()
    {
        this.mesh = GetComponent<MeshFilter>().mesh;
        this.vertices = this.mesh.vertices;
        this.triangles = this.mesh.triangles;
    }
    private void CreateNodes()
    {
        for (int i = 0; i < this.vertices.Length; ++i)
        {
            Vector3 position = this.transform.TransformPoint(vertices[i]);
            Collider fixer = GetFixer(position);
            bool is_fixed = false;
            if (fixer != null)
            {
                is_fixed = true;
            }
            Node current_node = new Node(position, this.node_mass, is_fixed);
            this.nodes.Add(current_node);
            
            if (is_fixed)
            {
                if (!this.fixers_info.ContainsKey(fixer))
                {
                    FixerInfo info = new FixerInfo(current_node.position);
                    this.fixers_info.Add(fixer, info);
                }
                this.fixers_info[fixer].fixed_nodes.Add(current_node);
            }
        }
    }


    /* Old implementation
    private void CreateNodes()
    {
        for (int i = 0; i < this.vertices.Length; ++i)
        {
            Vector3 position = this.transform.TransformPoint(vertices[i]);
            bool is_fixed = IsWithinAnyFixerBounds(position);
            Node current_node = new Node(position, this.node_mass, is_fixed);
            this.nodes.Add(current_node);
        }
    }
    */

    private void CreateEdgesAndSprings()
    {
        //note: triangles are stored as an array of integers, where each 3 ints correspond to the index of the vertices that make up said triangle.
        //this means that triangles.length has to be a multiple of 3.
        //this also means that vertices.length * 3 ought to be the number of elements of the triangles array.
        for (int i = 0; i < this.triangles.Length; i+=3)
        {
            int a = this.triangles[i + 0];
            int b = this.triangles[i + 1];
            int c = this.triangles[i + 2];

            CreateEdge(a,b,c);
            CreateEdge(c,a,b);
            CreateEdge(b,c,a);
        }
    }

    //Create an edge that connects vertices a and b, and store vertex c as the opposite to this edge.
    private void CreateEdge(int a, int b, int c)
    {
        Edge edge = new Edge(a, b);
        if (!this.edges.ContainsKey(edge))
        {
            this.edges.Add(edge, c);
            CreateSpringTraccion(a, b);
        }
        else
        {
            int opposite_vertex = edges[edge];
            Edge opposite_edge = new Edge(c, opposite_vertex);
            CreateSpringFlexion(c, edges[edge]);
        }
    }


    private void CreateSpringTraccion(int a, int b)
    {
        this.springs_traccion.Add(new Spring(this.nodes[a], this.nodes[b], this.spring_stiffness_traccion));
    }

    private void CreateSpringFlexion(int a, int b)
    {
        this.springs_flexion.Add(new Spring(this.nodes[a], this.nodes[b], this.spring_stiffness_flexion));
    }

    //updates the vertex positions of the mesh based on the node's position data.
    private void UpdateVertexPositions()
    {
        for (int i = 0; i < this.nodes.Count; ++i)
        {
            this.vertices[i] = this.transform.InverseTransformPoint(this.nodes[i].position);
        }
        this.mesh.vertices = vertices;
    }

    private void UpdateSurfaceNormals()
    {
        this.mesh.RecalculateNormals();
    }

    private bool IsWithinAnyFixerBounds(Vector3 position)
    {
        foreach (var fixer in this.fixers)
        {
            if (this.IsWithinFixerBounds(fixer, position))
                return true;
        }
        return false;
    }

    private Collider GetFixer(Vector3 position)
    {
        foreach (var fixer in this.fixers)
        {
            if (this.IsWithinFixerBounds(fixer, position))
                return fixer;
        }
        return null;
    }

    private bool IsWithinFixerBounds(Collider fixer, Vector3 position)
    {
        return fixer.bounds.Contains(position);
    }

    //Reminder: Wind force is calculated and applied over triangles rather than vertices so taht we can evenly distribute the wind force across the entire face surface to all vertices (for instance, 1/3 of the force applied to the surface is distributed to each vertex that makes up the triangle, etc...).
    //We could theoretically calculate it for every single vertex, but the reason we do so for every single triangle is because we think of the mesh as a surface, and the wind collides with multiple points at the same time, not with every single point individually, which means that the mesh is affected by the normal of the surface. Since we don't have infinitely many particles / points, we need to depend on surface normals for this. Or, we could store our own vertex normals on every single node, but we'll go with this implementation instead.
    //As it is currently implemented, the wind force is calculated to be a vector that goes in the direction of the surface normal. We calculate its force according to the wind force, and the angle between the wind vector and the surface normal (that's what the dot product is for!).
    private Vector3 CalculateWindForce(Node a, Node b, Node c, Vector3 wind_vector, float wind_force)
    {
        Vector3 v1 = b.position - a.position;
        Vector3 v2 = c.position - a.position;
        Vector3 cross_product = Vector3.Cross(v1, v2);

        Vector3 triangle_normal = cross_product.normalized;
        float triangle_area = cross_product.magnitude / 2;

        Vector3 ans = (Vector3.Dot(triangle_normal, wind_vector) * wind_force * triangle_area / 3) * triangle_normal;

        return ans;
    }

    //Extra notes about cross product:
    /*
        To calculate the normal of the triangle formed by nodes {A,B,C}, we can do the following: 
        
        Since we know that each surface is a flat triangle, we know that we can use 2 vectors that go from one point to another point each from the triangle to
        calculate the cross product and obtain a vector which will ne perpendicular to both of them, which we can normalize to obtain a valid normal vector.
        
        Steps to follow:
            1) calculate vector BA
            2) calculate vector CA
            3) cross product of BA and CA
            4) normalize the obtained vector and return it
        
        EZ.

        Other important info obtained from cross product is:
            -area of the polygon defined by the 2 vectors (if we divide this by 2, we get the area of a triangle. Again, ez).
    */

    private void SimulateWindForces()
    {
        for (int i = 0; i < this.triangles.Length; i += 3)
        {
            var node0 = this.nodes[this.triangles[i + 0]];
            var node1 = this.nodes[this.triangles[i + 1]];
            var node2 = this.nodes[this.triangles[i + 2]];
            Vector3 wind_force_per_node = CalculateWindForce(node0, node1, node2, this.wind_vector, this.wind_force);
            node0.AddForceVector(wind_force_per_node);
            node1.AddForceVector(wind_force_per_node);
            node2.AddForceVector(wind_force_per_node);
        }
    }

    private void DebugDraw()
    {
        print($"Nodos: {this.nodes.Count}; Muelles de tracción: {this.springs_traccion.Count}; Muelles de flexión: {this.springs_flexion.Count}; Muelles en total: {this.springs_traccion.Count + this.springs_flexion.Count}");
        foreach (var spring in this.springs_traccion)
        {
            Debug.DrawLine(spring.node_a.position, spring.node_b.position, Color.red);
        }
        foreach (var spring in this.springs_flexion)
        {
            Debug.DrawLine(spring.node_a.position, spring.node_b.position, Color.blue);
        }
    }

    private Vector3 CalculatePenetrationForce(SphereCollider collider, Vector3 point, float penetration_distance_margin, float penetration_force)
    {
        Vector3 force = Vector3.zero;

        Vector3 vector_director = collider.transform.position - point;

        float penetration_distance = vector_director.magnitude;

        float max_scale = Mathf.Max(collider.transform.localScale.x, collider.transform.localScale.y, collider.transform.localScale.z);

        if ((penetration_distance + penetration_distance_margin) < (collider.radius * max_scale))
        {
            Vector3 vector_normal = vector_director.normalized;
            force = penetration_force * penetration_distance * vector_normal;
        }
        return force;
    }

    private void CalculatePenaltyForce(SphereCollider collider, Node node)
    {
        Vector3 force = CalculatePenetrationForce(collider, node.position, this.penalty_penetration_distance_margin, this.penalty_force);
        node.force -= force;
    }

    private void CalculatePullForce(SphereCollider collider, Node node)
    {
        Vector3 force = CalculatePenetrationForce(collider, node.position, this.pull_penetration_distance_margin, this.pull_force);
        node.force += force;
    }

    private void SimulatePenaltyForces()
    {
        foreach (var node in this.nodes)
        {
            foreach (var collider in this.colliders_penalty_forces)
                this.CalculatePenaltyForce(collider, node);
        }
    }

    private void SimulatePullForces()
    {
        foreach (var node in this.nodes)
        {
            foreach (var collider in this.colliders_pull_forces)
                this.CalculatePullForce(collider, node);
        }
    }

    #endregion
}
