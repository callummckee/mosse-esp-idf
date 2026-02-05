#pragma once

#include <initializer_list>
#include <algorithm> //for std::copy

template<int R, int C>
class Mat {
    public:
        float m_data[R*C];
        int m_length = R*C;
        Mat(std::initializer_list<float> list) {
            std::copy(list.begin(), list.end(), m_data);
        }

        void invert2x2(Mat<2, 2>& out) const {
            // inverts matrix and sends output to out, if det = 0, then each element in output set to 0
            static_assert(R == 2 && C == 2, "invert2x2 must be used on a 2x2 Mat!");
            float det = (this->m_data[0] * this->m_data[3] - this->m_data[1] * this->m_data[2]);
            if (det < 1e-6) {
                out = {0, 0, 0, 0}; //extra copy
                return; 
            }
            float rec_det = 1.0/det;
            out.m_data[0] = this->m_data[3] * rec_det;
            out.m_data[1] = this->m_data[1] * rec_det * -1;
            out.m_data[2] = this->m_data[2] * rec_det * -1;
            out.m_data[3] = this->m_data[0] * rec_det;
        }

        void vecMult2D(const Mat<2,1>& in_vec, Mat<2, 1>& out_vec) const {
            static_assert(R == 2 && C == 2, "vecMult2D must be used on a 2x2 Mat!");
            out_vec.m_data[0] = this->m_data[0] * in_vec.m_data[0] + this->m_data[1] * in_vec.m_data[1];
            out_vec.m_data[1] = this->m_data[2] * in_vec.m_data[0] + this->m_data[3] * in_vec.m_data[1];
        }

        bool isZero() const {
            //returns true if every element in the Mat is zero
            bool check = 1;
            for (int i = 0; i < this->m_length; i++) {
                check = check && (this->m_data[i] == 0);
            }
            return check;
        }
};
