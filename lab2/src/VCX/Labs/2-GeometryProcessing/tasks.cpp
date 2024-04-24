#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                // your code here:
                int n=neighbors.size();
                float u;
                if(n==3)u=3.0/16.0;
                else u=3.0/(8.0*n);
                glm::vec3 sum={0.0f,0.0f,0.0f};
                for(std::size_t j=0;j<neighbors.size();++j){
                    sum+=prev_mesh.Positions[neighbors[j]];
                }
                sum*=u;
                sum+=(1.0f-n*u)*prev_mesh.Positions[i];
                curr_mesh.Positions.push_back(sum);
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                //printf("%d %d %d\n",G.IndexOf(e->Face()),e->EdgeLabel(),curr_mesh.Positions.size());
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 new_vertex=0.5f*(prev_mesh.Positions[e->To()]+prev_mesh.Positions[e->From()]);
                    curr_mesh.Positions.push_back(new_vertex);
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    //printf("%d %d %d\n",G.IndexOf(eTwin->Face()),e->TwinEdge()->EdgeLabel(),curr_mesh.Positions.size());
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 new_vertex=0.375f*(prev_mesh.Positions[e->To()]+prev_mesh.Positions[e->From()])+0.125f*(prev_mesh.Positions[e->OppositeVertex()]+prev_mesh.Positions[e->TwinOppositeVertex()]);
                    curr_mesh.Positions.push_back(new_vertex);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                //printf("%d %d %d\n")
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    v0,m2,m1,
                    v1,m0,m2,
                    v2,m1,m0,
                    m0,m1,m2                    
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
            /*printf("%zd %zd %zd %zd\n",prev_mesh.Positions.size(),curr_mesh.Positions.size(),prev_mesh.Indices.size()/3,curr_mesh.Indices.size()/3);
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U){
                printf("%d %d %d\n",prev_mesh.Indices[i + 0U],prev_mesh.Indices[i + 1U],prev_mesh.Indices[i + 2U]);
            }
            printf("sjk\n");
            for (std::size_t i = 0; i < curr_mesh.Indices.size(); i += 3U){
                printf("%d %d %d\n",curr_mesh.Indices[i + 0U],curr_mesh.Indices[i + 1U],curr_mesh.Indices[i + 2U]);
            }*/
            
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        
        for(std::size_t i = 0; i < input.Positions.size(); ++i){
            if(G.Vertex(i)->OnBoundary()){
                //printf("%lf %lf %lf\n",input.Positions[i][0],input.Positions[i][1],input.Positions[i][2]);
                glm::vec2 boud={input.Positions[i][0],input.Positions[i][1]};
                output.TexCoords[i]=glm::normalize(boud)/2.0f+0.5f;
               // printf("%lf %lf\n",output.TexCoords[i][0],output.TexCoords[i][1]);
            }
           
        }

        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            for(std::size_t i = 0; i < input.Positions.size(); ++i){
                if(G.Vertex(i)->OnBoundary())continue;
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                int n=neighbors.size();
                glm::vec2 sum={0.0f,0.0f};
                for(std::size_t j=0;j<neighbors.size();++j)
                    sum+=output.TexCoords[neighbors[j]];
                output.TexCoords[i]=sum/(float)n;        
            }
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Q matrix of the face f.
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Q;
                // your code here:
                glm::mat3 pos={
                    output.Positions[f->VertexIndex(0)],
                    output.Positions[f->VertexIndex(1)],
                    output.Positions[f->VertexIndex(2)]};
                pos=glm::transpose(pos);
                glm::vec3 par={-1.0f,-1.0f,-1.0f};
                par=glm::inverse(pos)*par;
                float t=sqrt(glm::dot(par,par));
                glm::vec4 q{par,1.0f};
                if(t>=10000)return glm::mat4(0);
                q*=1.0f/t;
                Q= glm::mat4{ 
                    q[0]*q[0],q[0]*q[1],q[0]*q[2],q[0]*q[3],
                    q[1]*q[0],q[1]*q[1],q[1]*q[2],q[1]*q[3],
                    q[2]*q[0],q[2]*q[1],q[2]*q[2],q[2]*q[3],
                    q[3]*q[0],q[3]*q[1],q[3]*q[2],q[3]*q[3]
                };
                return Q;
            }
        };

        // The struct to record constraction info.
        struct ConstractionPair {
            DCEL::HalfEdge const * edge;            // which edge to constract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ConstractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ConstractionPair {
                // your code here:
                ConstractionPair out;
                out.edge=edge;
                glm::vec4 b{0.0f,0.0f,0.0f,1.0f};
                glm::mat4 const & Qq={
                    Q[0][0],Q[1][0],Q[2][0],b[0],
                    Q[0][1],Q[1][1],Q[2][1],b[1],
                    Q[0][2],Q[1][2],Q[2][2],b[2],
                    Q[0][3],Q[1][3],Q[2][3],b[3],
                };
                if(glm::determinant(Qq)>0.001f){
                    glm::vec4 targetPosition=glm::inverse(Qq)*b;
                    out.targetPosition=targetPosition;
                    out.cost=glm::dot(targetPosition,Q*targetPosition);
                }
                else{
                    glm::vec4 to{p1,1.0f};
                    glm::vec4 from{p2,1.0f};
                    glm::vec4 mid=(to+from)*0.5f;
                    float q1=glm::dot(to,Q*to),q2=glm::dot(from,Q*from),q3=glm::dot(mid,Q*mid);
                    if(q2<=q1){
                        out.targetPosition=from;
                        out.cost=q2;
                    }
                    else{
                        out.targetPosition=to;
                        out.cost=q1;
                    }
                    if(q3<out.cost){
                        out.targetPosition=mid;
                        out.cost=q3;
                    }   
                }
                return out;
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ConstractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Qf:       $Qf[idx]$ is the Q matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ConstractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Qf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Qf[G.IndexOf(f)]       = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the constractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsConstractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the constractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsConstractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the constractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the constract result
            // ring:   the edge ring of vertex v1
            ConstractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Constract(top.edge);
            auto               ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The constraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Qf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Qf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Q matrix for $e->Face()$.
                //     2. According to the difference between the old Q (in $Qf$) and the new Q (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                //     4. Update $Qf$.
                auto Qnew=UpdateQ(e->Face());
                auto Qold=Qf[G.IndexOf(e->Face())];
                Qv[e->From()]+=Qnew-Qold;
                Qv[e->To()]+=Qnew-Qold;
                Qv[v1]+=Qnew;
                Qf[G.IndexOf(e->Face())]=Qnew;
            }

            // Finally, as the Q matrix changed, we should update the relative $ConstractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
            for (auto e : ring) {
                auto v2=e->From();
                auto ring2   = G.Vertex(v2)->Ring();
                for (auto e2:ring2){
                    if (! G.IsConstractable(e2->NextEdge())){
                        pairs[pair_map[G.IndexOf(e2->NextEdge())]].edge=nullptr;
                    }
                    else{
                        auto vf= e2->To();
                        auto pair11= MakePair(e2->NextEdge(), output.Positions[vf], output.Positions[v2], Qv[vf] + Qv[v2]);
                        pairs[pair_map[G.IndexOf(e2->NextEdge())]].targetPosition=pair11.targetPosition;
                        pairs[pair_map[G.IndexOf(e2->NextEdge())]].cost=pair11.cost;
                    }
                }
            }

        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                float cos=glm::dot(v1-vAngle, v2-vAngle);
                float sin=sqrt(glm::dot(glm::cross(v1-vAngle, v2-vAngle),glm::cross(v1-vAngle, v2-vAngle)));
                float all=sqrt(cos*cos+sin*sin);
                if(sin/all<1.0e-2f){
                    sin=1.0e-2f*all;
                }
                return abs(cos/sin);
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...              
                if(useUniformWeight){
                    auto v           = G.Vertex(i);
                    auto neighbors   = v->Neighbors();
                    int n=neighbors.size();
                    glm::vec3 sum={0.0f,0.0f,0.0f};
                    for(std::size_t j=0;j<neighbors.size();++j)
                        sum+=prev_mesh.Positions[neighbors[j]];
                    sum/=n*1.0;
                    curr_mesh.Positions[i]=(1.0f-lambda)*prev_mesh.Positions[i]+lambda*sum;
                }
                else{
                    auto v           = G.Vertex(i);
                    auto oppohalfedge=v->Ring();
                    int n=oppohalfedge.size();
                    glm::vec3 sum={0.0f,0.0f,0.0f};
                    float sum1=0.0f;
                    for(std::size_t j=0;j<oppohalfedge.size();++j){
                        auto e=oppohalfedge[j];
                        auto v1=e->To();
                        auto v2=e->From();
                        float cotan1=GetCotangent(prev_mesh.Positions[v1],prev_mesh.Positions[i],prev_mesh.Positions[v2]);
                        float cotan2=GetCotangent(prev_mesh.Positions[v2],prev_mesh.Positions[i],prev_mesh.Positions[v1]);
                        //printf("%f %f\n",cotan1,cotan2);
                        sum+=cotan1*prev_mesh.Positions[v2]+cotan2*prev_mesh.Positions[v1];
                        sum1+=cotan1+cotan2;
                    }
                    sum/=sum1;
                    curr_mesh.Positions[i]=(1.0f-lambda)*prev_mesh.Positions[i]+lambda*sum;
                }
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }
    short edge_dic[200][200][200][3];
    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:              
        memset(edge_dic, -1, sizeof(edge_dic));
        glm::vec3 unit[3];
        unit[0]=glm::vec3{1.0f,0.0f,0.0f};
        unit[1]=glm::vec3{0.0f,1.0f,0.0f};
        unit[2]=glm::vec3{0.0f,0.0f,1.0f};
        for(int x=0;x<n;x++){
            for(int y=0;y<n;y++){
                for(int z=0;z<n;z++){
                    int cube_index=0;
                    glm::vec3 pos_i={grid_min[0]+x*dx,
                                        grid_min[1]+y*dx,
                                        grid_min[2]+z*dx};
                    float dis;
                    for(int i=0;i<8;i++){
                        glm::vec3 pos=glm::vec3{pos_i[0]+(i&1)*dx,
                                    pos_i[1]+((i>>1)&1)*dx,
                                    pos_i[2]+(i>>2)*dx};
                        dis=sdf(pos);
                        if(dis>0)
                            cube_index|=1<<i;
                    }
                   // printf("%d %d %d %d\n",x,y,z,cube_index);
                    auto cube_edge_index=c_EdgeStateTable[cube_index];
                    if(cube_edge_index==0)continue;
                    
                    //
                    int flag_x=0,flag_y=0,flag_z=0;
                    int add_num=0;
                    for(int i=0; i<12;i++){
                        if((cube_edge_index&(1<<i)) == 0)
                            continue;
                        flag_x=0,flag_y=0,flag_z=0;
                        if(i==6||i==7||i==9||i==11)flag_x=1;
                        if(i==1||i==3||i==10||i==11)flag_y=1;
                        if(i==2||i==3||i==5||i==7)flag_z=1;
                        add_num++;
                        if(edge_dic[x+flag_x][y+flag_y][z+flag_z][i/4]!=-1)
                            continue;
                        edge_dic[x+flag_x][y+flag_y][z+flag_z][i/4]=output.Positions.size();                   
                        glm::vec3 position;
                        int from,to;
                        glm::vec3 from_position=pos_i+dx*(i&1)*unit[((i>>2)+1)%3]+dx*((i>>1)&1)*unit[((i>>2)+2)%3];
                        glm::vec3 to_position=from_position+unit[i>>2]*dx;
                        float to_dis=sdf(to_position),from_dis=sdf(from_position);
                        
                        position=(to_dis*from_position-from_dis*to_position)/(to_dis-from_dis);
                        output.Positions.push_back(position);
                        
                        //printf("%d %d %d %d %d %f %f\n",x,y,z,to,from,to_dis,from_dis);
                    }
                    // connect
                    
                    //auto cube_face_index=c_EdgeOrdsTable[cube_index];
                    for(int i=0;i<add_num-2;i++){
                        for(int j=0;j<3;j++){
                            int c=c_EdgeOrdsTable[cube_index][3*i+2-j];
                            if(c==-1)break;
                            //printf("%d %d %d %d\n",x,y,z,c);
                            flag_x=0,flag_y=0,flag_z=0;
                            if(c==6||c==7||c==9||c==11)flag_x=1;
                            if(c==1||c==3||c==10||c==11)flag_y=1;
                            if(c==2||c==3||c==5||c==7)flag_z=1;
                            output.Indices.push_back(edge_dic[x+flag_x][y+flag_y][z+flag_z][c/4]);
                            
                        }
                        
                    }
                }
            }
        }
        
    }
} // namespace VCX::Labs::GeometryProcessing
