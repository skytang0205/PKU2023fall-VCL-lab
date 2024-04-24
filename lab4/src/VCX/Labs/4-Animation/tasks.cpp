#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i-1]*ik.JointLocalRotation[i];
            glm::vec4 p               = glm::vec4(ik.JointLocalOffset[i], 1.0f);
            p                         = glm::mat4_cast(ik.JointGlobalRotation[i - 1]) * p;
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i-1]+glm::vec3(p.x,p.y,p.z)/p.w;
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            int len=ik.JointLocalOffset.size();
            for(int i=len-2;i>=0;i--){
                glm::vec3 dirt = glm::normalize(EndPosition-ik.JointGlobalPosition[i]);
                glm::vec3 end  = glm::normalize(ik.JointGlobalPosition[len-1]-ik.JointGlobalPosition[i]);
                ik.JointLocalRotation[i] = glm::rotation(end,dirt) * ik.JointLocalRotation[i];
                ForwardKinematics(ik, i);
            }
        }
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                backward_positions[i]=backward_positions[i+1]+glm::normalize(ik.JointGlobalPosition[i]-backward_positions[i+1])*ik.JointOffsetLength[i+1];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                forward_positions[i+1]=forward_positions[i]+glm::normalize(backward_positions[i+1]-forward_positions[i])*ik.JointOffsetLength[i+1];
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
        int nums = 3500;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(5000));
        int index = 0;
        float last_x,last_y;
        float maxlen=0.02f;
        /*
        for (int i = 0; i < nums; i++) {
            
            float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
            float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
            if(i==0){
                last_x=x_val;
                last_y=y_val;
                if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
                (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
                continue;
            }
            float len=sqrt((x_val-last_x)*(x_val-last_x)+(y_val-last_y)*(y_val-last_y));
            if(len/maxlen<2){
                last_x=x_val;
                last_y=y_val;
                if (std::abs(x_val) < 1e-3f || std::abs(y_val) < 1e-3f) continue;
                (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);           
                continue;
            }
            for(int j=(int)(len/maxlen);j>=0;j--){
                x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * (i*1.0-j*1.0/(int)(len/maxlen)) / nums);
                y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * (i*1.0-j*1.0/(int)(len/maxlen)) / nums);
                if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
                (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
            }     
            last_x=x_val;
            last_y=y_val;
        }
        //printf("%d",index);
        custom->resize(index);
        return custom;
        */
        nums=100;
        float pi=glm::pi<float>();
        // S
        for (int i = 0; i < nums; ++i) {
            float x_val        = cos(1.5f*pi*i/nums);
            float y_val        = sin(1.5f*pi*i/nums);
            (*custom)[index++] = glm::vec3(-0.2f-0.2f*x_val,0.0f,0.375f+0.125f*y_val);
        }

        for (int i = 0; i < nums; i++) {
            float x_val        = cos(-1.5f*pi*i/nums+0.5f*pi);
            float y_val        = sin(-1.5f*pi*i/nums+0.5f*pi);
            (*custom)[index++] = glm::vec3(-0.2f-0.2f*x_val,0.0f,0.125f+0.125f*y_val);
        }
        // K
        for (int i = 0; i < nums; i++) {
            float x_val        = 0.5;
            float y_val        = 0.5*i/nums;
            (*custom)[index++] = glm::vec3(-x_val, 0.0f, y_val);
        }

        for (int i = 0; i < nums; ++i) {
            float x_val        = 0.9-0.4*i/nums;
            float y_val        = 0.5-0.25*i/nums;
            (*custom)[index++] = glm::vec3(-x_val, 0.0f, y_val);
        }

        for (int i = 0; i < nums; ++i) {
            float x_val        = 0.5+0.4*i/nums;
            float y_val        = 0.25-0.25*i/nums;
            (*custom)[index++] = glm::vec3(-x_val, 0.0f, y_val);
        }
        // Y  
        for (int i = 0; i < nums; ++i) {
            float x_val        = 1.0+0.2*i/nums;
            float y_val        = 0.5-0.25*i/nums;
            (*custom)[index++] = glm::vec3(-x_val, 0.0f, y_val);
        }

        for (int i = 0; i < nums; ++i) {
            float x_val        = 1.4-0.2*i/nums;
            float y_val        = 0.5-0.25*i/nums;
            (*custom)[index++] = glm::vec3(-x_val, 0.0f, y_val);
        }

        for (int i = 0; i < nums; i++) {
            float x_val        = 1.2;
            float y_val        = 0.25*i/nums;
            (*custom)[index++] = glm::vec3(-x_val, 0.0f, y_val);
        }
        custom->resize(index);
        return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    static Eigen::VectorXf grav(MassSpringSystem & system){
        std::vector<glm::vec3> gravity(system.Positions.size(), glm::vec3(0, system.Gravity, 0));
        return glm2eigen(gravity);
    }

    static Eigen::VectorXf damp_force(MassSpringSystem & system, Eigen::VectorXf const & x, Eigen::VectorXf const & v){
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        std::vector<glm::vec3> vec_x = eigen2glm(x);
        std::vector<glm::vec3> vec_v = eigen2glm(v);
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = vec_x[p1] - vec_x[p0];
            glm::vec3 const v01 = vec_v[p1] - vec_v[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = system.Damping * glm::dot(v01, e01) * e01;
            forces[p0] += f;
            forces[p1] -= f;
        }
        return glm2eigen(forces);
    }

    static Eigen::VectorXf grad_E(MassSpringSystem & system, Eigen::VectorXf const & x){
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
        std::vector<glm::vec3> vec_x = eigen2glm(x);
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = vec_x[p1] - vec_x[p0];
            glm::vec3 const e01 = glm::normalize(x01);
            glm::vec3 f = system.Stiffness * (spring.RestLength - glm::length(x01)) * e01;
            forces[p0] += f;
            forces[p1] -= f;
        }
        return glm2eigen(forces);
    }

    static Eigen::VectorXf y(MassSpringSystem & system, Eigen::VectorXf const & x, Eigen::VectorXf const & v, float const ddt){
        return x + ddt * v + (ddt * ddt / system.Mass) * damp_force(system, x, v) - ddt * ddt * grav(system);
    }

    static Eigen::VectorXf grad_g(MassSpringSystem & system, Eigen::VectorXf const & x, Eigen::VectorXf const & v, float const ddt){
        return (system.Mass / (ddt * ddt)) * (x - y(system, x, v, ddt)) + grad_E(system, x);
    }

    static Eigen::SparseMatrix<float> Hessian(MassSpringSystem & system, Eigen::VectorXf const & x, float const ddt){
        std::vector<Eigen::Triplet<float>> triplets;
        std::vector<glm::vec3> vec_x = eigen2glm(x);
        float m=system.Mass/(ddt*ddt);
        std::vector<glm::mat3> diagonal(system.Positions.size(),glm::mat3(m, 0.0f, 0.0f, 0.0f, m, 0.0f, 0.0f, 0.0f, m));        
        for (auto const spring : system.Springs) {
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const x01 = vec_x[p1] - vec_x[p0];
            float len=glm::length(x01);
            float p;
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    if(i==j) p=system.Stiffness*((x01[i]*x01[j])/(len*len)+(1.0-spring.RestLength/len)*(1.0-(x01[i]*x01[j])/(len*len)));
                    else p=system.Stiffness*((x01[i]*x01[j])/(len*len)+(1.0-spring.RestLength/len)*(0.0-(x01[i]*x01[j])/(len*len)));                    
                    diagonal[p0][i][j]+=p;
                    diagonal[p1][i][j]+=p;
                    triplets.emplace_back(Eigen::Triplet<float>(3*p0+i,3*p1+j,-p));
                    triplets.emplace_back(Eigen::Triplet<float>(3*p1+i,3*p0+j,-p));
                }
            }
        }
        for(std::size_t k=0;k<system.Positions.size();k++){
            for(int i=0;i<3;i++){
                for(int j=0;j<3;j++){
                    triplets.emplace_back(Eigen::Triplet<float>(3*k+i,3*k+j,diagonal[k][i][j]));
                }
            }
        }
        return CreateEigenSparseMatrix(3*system.Positions.size(),triplets);
    }


    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        // your code here: rewrite following code
        int const steps = 10;
        float const ddt = dt / steps; 
        /*
        for (std::size_t s = 0; s < steps; s++) {
            std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
            for (auto const spring : system.Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
                glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
                glm::vec3 const e01 = glm::normalize(x01);
                glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
                forces[p0] += f;
                forces[p1] -= f;
            }
            for (std::size_t i = 0; i < system.Positions.size(); i++) {
                if (system.Fixed[i]) continue;
                system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
                system.Positions[i] += system.Velocities[i] * ddt;
            }
        }*/
        Eigen::VectorXf x=glm2eigen(system.Positions);
        Eigen::VectorXf v=glm2eigen(system.Velocities);
        for (std::size_t s = 0; s < steps; s++){
            x += ComputeSimplicialLLT(Hessian(system,x,ddt),-grad_g(system,x,v,ddt));
            v += ((damp_force(system,x,v)-grad_E(system,x))/system.Mass-grav(system))*ddt;
        }
        auto newx=eigen2glm(x),newv=eigen2glm(v);
        for (std::size_t i = 0; i < system.Positions.size(); i++) {
            if (system.Fixed[i]) continue;
            system.Velocities[i] = newv[i];
            system.Positions[i]  = newx[i];
        }
    }
}
