// ˵��: �ռ���������Ĳ���

#ifndef __G_QTREE__
#define __G_QTREE__

#include <cstddef>
#include <vector>
#include <limits>
#include <cmath>

static const double EPS = 0.0000000001;

namespace GZXL
{
#define PI 3.14159265358979323846

    // �Ĳ��� ģ�嶨��
    template<typename COOR_TYPE, typename DATA_TYPE> class GQtree;

    // �Ĳ����ڵ㶨��
    template<typename COOR_TYPE, typename DATA_TYPE> class node
    {
    public:
        COOR_TYPE x;             // ���ε�����x
        COOR_TYPE y;             // ���ε�����y
        COOR_TYPE r;             // �뾶
        COOR_TYPE w;             // ���ο��(�����½�Ϊ����)
        COOR_TYPE h;             // ���θ߶�(�����½�Ϊ����)

        COOR_TYPE xs;            // ֱ�����x
        COOR_TYPE ys;            // ֱ�����y
        COOR_TYPE xe;            // ֱ���յ�x
        COOR_TYPE ye;            // ֱ���յ�y
        COOR_TYPE tol;           // �ݲ�

        DATA_TYPE data;          // �û����������

    private:
        friend class GQtree<COOR_TYPE, DATA_TYPE>;
        // �ڵ�� ���� ���� ���� ����
        explicit node(COOR_TYPE left, COOR_TYPE top, COOR_TYPE right, COOR_TYPE bottom)
        {
            x = left;
            y = top;
            x1 = right;
            y1 = bottom;
            leaf = false;
            child[0] = child[1] = child[2] = child[3] = NULL;
            node_size = 0;
        }
        // 2D��(x y)
        explicit node(COOR_TYPE x, COOR_TYPE y, DATA_TYPE data)
        {
            this->x = x;
            this->y = y;
            this->data = data;
            leaf = true;
        }
        // Բ(x y �뾶r)
        explicit node(COOR_TYPE x, COOR_TYPE y, COOR_TYPE r, DATA_TYPE data)
        {
            this->x = x;
            this->y = y;
            this->r = r;
            this->data = data;
            leaf = true;
        }
        // ����(x y ��w ��h)
        explicit node(COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h, DATA_TYPE data)
        {
            this->x = x;
            this->y = y;
             //this->r = r;
            this->w = w;
            this->h = h;
            this->data = data;
            leaf = true;
        }
        // ֱ��(xs ys xe ye tol data)
        explicit node(COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye, COOR_TYPE tol, DATA_TYPE data)
        {
            this->xs = xs;
            this->ys = ys;
            this->xe = xe;
            this->ye = ye;
            this->tol = tol;
            this->data = data;
            leaf = true;
        }

        COOR_TYPE x1;
        COOR_TYPE y1;
        node* child[4];
        bool leaf;
        size_t node_size;
    };

    // �Ĳ�������
    template<typename COOR_TYPE, typename DATA_TYPE> class GQtree
    {
    public:
        typedef node<COOR_TYPE, DATA_TYPE> nodeType;

        // ͨ��ָ�����α߽��4�����������Ĳ���
        explicit GQtree(COOR_TYPE left, COOR_TYPE top, COOR_TYPE right, COOR_TYPE bottom)
        {
            root = new nodeType(left, top, right, bottom);
        }

        ~GQtree()
        {
            // �ͷŽڵ�
            ReleaseNode(root);
        }

        // ��ӽڵ�(2D�� x y)
        bool AddNode(COOR_TYPE x, COOR_TYPE y, DATA_TYPE data)
        {
            return AddNode(root, x, y, data);
        }
        // ��ӽڵ�(Բ x y �뾶r)
        bool AddNode(COOR_TYPE x, COOR_TYPE y, COOR_TYPE r, DATA_TYPE data)
        {
            return AddNode(root, x, y, r, data);
        }
        // ��ӽڵ�(���� x y ��w ��h)
        bool AddNode(COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h, DATA_TYPE data)
        {
            return AddNode(root, x, y, w, h, data);
        }
        // ���ֱ��(xs ys ns xe ye tol data)
        bool AddNode(COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye, COOR_TYPE tol, DATA_TYPE data)
        {
            return AddNode(root, xs, ys, xe, ye, tol, data);
        }

        // ɾ���ڵ�(2D�� x y)
        bool DeleteNode(COOR_TYPE x, COOR_TYPE y, DATA_TYPE data)
        {
            return DeleteNode(root, x, y, data);
        }
        // ɾ���ڵ�(Բ x y �뾶r)
        bool DeleteNode(COOR_TYPE x, COOR_TYPE y, COOR_TYPE r, DATA_TYPE data)
        {
            return DeleteNode(root, x, y, r, data);
        }
        // ɾ���ڵ�(���� x y ��w ��h)
        bool DeleteNode(COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h, DATA_TYPE data)
        {
            return DeleteNode(root, x, y, w, h, data);
        }
        // ɾ���ڵ�(ֱ�� xs ys ns xe ye tol data)
        bool DeleteNode(COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye, COOR_TYPE tol, DATA_TYPE data)
        {
            return DeleteNode(root, xs, ys, xe, ye, tol, data);
        }

        // ���������Բ�ڵ�(���x yֵ �뾶)
        const nodeType* FindNearest(COOR_TYPE x, COOR_TYPE y, COOR_TYPE radius) const
        {
            std::vector<const nodeType*> result;
            if (Search(x, y, radius, result))
            {
                int index = 0;
                COOR_TYPE d = (result[0]->x - x) * (result[0]->x - x) + (result[0]->y - y) * (result[0]->y - y);
                for (int i = 1; i < result.size(); i++)
                {
                    COOR_TYPE d1 = (result[i]->x - x) * (result[i]->x - x) + (result[i]->y - y) * (result[i]->y - y);
                    if (d1 < d)
                    {
                        d = d1;
                        index = i;
                    }
                }
                return result[index];
            }
            else
            {
                return NULL;
            }
        }

        // ��������ľ��νڵ�(���x yֵ ��w ��h)
        const nodeType* FindNearest(COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h) const
        {
            std::vector<const nodeType*> result;
            if (Search(x, y, w, h, result))
            {
                int index = 0;
                COOR_TYPE d = (result[0]->x - x) * (result[0]->x - x) + (result[0]->y - y) * (result[0]->y - y);
                for (int i = 1; i < result.size(); i++)
                {
                    COOR_TYPE d1 = (result[i]->x - x) * (result[i]->x - x) + (result[i]->y - y) * (result[i]->y - y);
                    if (d1 < d)
                    {
                        d = d1;
                        index = i;
                    }
                }
                return result[index];
            }
            else
            {
                return NULL;
            }
        }

        // ����ָ���뾶�ڵĽڵ�(���x yֵ �뾶)
        bool Search(COOR_TYPE x, COOR_TYPE y, COOR_TYPE radius, std::vector<const nodeType*>& result) const
        {
            result.clear();
            Search(root, x, y, radius, result);
            return !result.empty();
        }

        // ����ָ������ڵĽڵ�(���x yֵ ��w ��h)
        bool Search(COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h, std::vector<const nodeType*>& result) const
        {
            result.clear();
            Search(root, x, y, w, h, result);
            return !result.empty();
        }

        // �����ֱཻ��(�����ݲ�) (xs:���x ys:���y xe:�յ�x ye:�յ�y)
        bool SearchLine(COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye, std::vector<const nodeType*>& result) const
        {
            result.clear();
            SearchLine(root, xs, ys, xe, ye, result);
            return !result.empty();
        }

        // �����ֱཻ��(���ݲ�) (xs:���x ys:���y xe:�յ�x ye:�յ�y tol:�ݲ�ֵ)
        bool SearchLine(COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye, COOR_TYPE tol,
            std::vector<const nodeType*>& result) const
        {
            result.clear();
            SearchLine(root, xs, ys, xe, ye, tol, result);
            return !result.empty();
        }

    private:
        nodeType* root;

        GQtree(const GQtree& other) {} //noncopyable
        GQtree& operator=(const GQtree&) {}
        
        // �Ƿ��ڽڵ�ķ�Χ��
        bool InReach(COOR_TYPE x0, COOR_TYPE y0, COOR_TYPE x1, COOR_TYPE y1, COOR_TYPE r) const
        {
            return (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) <= r * r;
        }

        // �Ƿ��ڽڵ�ķ�Χ��
        bool InReach(COOR_TYPE x0, COOR_TYPE y0, COOR_TYPE x1, COOR_TYPE y1, COOR_TYPE w, COOR_TYPE h) const
        {
            return (x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) <= w * h;
        }
        
        // ��Բ�Ƿ��ཻ
        bool Intersect(const nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE r) const
        {
            COOR_TYPE x0 = (n->x + n->x1) / 2;
            COOR_TYPE y0 = (n->y + n->y1) / 2;
            COOR_TYPE vx = x > x0 ? x - x0 : x0 - x;
            COOR_TYPE vy = y > y0 ? y - y0 : y0 - y;
            COOR_TYPE hx = n->x > x0 ? n->x - x0 : x0 - n->x;
            COOR_TYPE hy = n->y > y0 ? n->y - y0 : y0 - n->y;
            COOR_TYPE ux = vx > hx ? vx - hx : 0;
            COOR_TYPE uy = vy > hy ? vy - hy : 0;
            return ux * ux + uy * uy <= r * r;
			//COOR_TYPE delta = n->r + r;
			//COOR_TYPE left = n->x - delta;
			//COOR_TYPE right = n->x1 + delta;
			//COOR_TYPE top = n->y1 + delta;
			//COOR_TYPE bottom = n->y - delta;
			//if (Inbound(left, bottom, right, top, x, y)) return true;
			//else return false;

			////// ��ϸɸѡ����
			////if (Between(left, right, x) && Between(n->y, n->y1, y)) return true;
			////if (Between(bottom, top, y) && Between(n->x, n->x1, x)) return true;
			////if (InReach(n->x, n->y, x, y, delta) || InReach(n->x1, n->y, x, y, delta) || InReach(n->x1, n->y1, x, y, delta) || InReach(n->x, n->y1, x, y, delta)) return true;
			////else return false;
        }

        // �ж�����Բ�Ƿ��ཻ
        bool Intersect(COOR_TYPE x1, COOR_TYPE y1, COOR_TYPE r1, COOR_TYPE x2, COOR_TYPE y2, COOR_TYPE r2) const
        {
            COOR_TYPE d_r = r1 + r2;
            COOR_TYPE d_x = x1 - x2;
            COOR_TYPE d_y = y1 - y2;
            COOR_TYPE d_distance = sqrt(d_x * d_x + d_y * d_y);
            if (d_distance > d_r)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        // ֱ����ֱ���ཻ(�����ݲ�)
        bool LineLineIntersect(COOR_TYPE xs1, COOR_TYPE ys1, COOR_TYPE xe1, COOR_TYPE ye1,
            COOR_TYPE xs2, COOR_TYPE ys2, COOR_TYPE xe2, COOR_TYPE ye2) const
        {
            COOR_TYPE mua, mub;

            COOR_TYPE denom = (ye2 - ys2) * (xe1 - xs1) - (xe2 - xs2) * (ye1 - ys1);
            COOR_TYPE numera = (xe2 - xs2) * (ys1 - ys2) - (ye2 - ys2) * (xs1 - xs2);
            COOR_TYPE numerb = (xe1 - xs1) * (ys1 - ys2) - (ye1 - ys1) * (xs1 - xs2);

            // �����Ƿ��غ�
            if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS)
            {
                return true;
            }

            // ���Ƿ�ƽ��
            if (fabs(denom) < EPS)
            {
                return false;
            }

            // �Ƿ������߶εĽ���
            mua = numera / denom;
            mub = numerb / denom;
            if (mua < 0 || mua > 1 || mub < 0 || mub > 1)
            {
                return false;
            }

            return true;
        }

        // ֱ����ֱ���ཻ(�����ݲ�)(xs:���x ys:���y xe:�յ�x ye:�յ�y)
        bool LineLineIntersect(const nodeType* n, COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye) const
        {
            COOR_TYPE mua, mub;
            COOR_TYPE denom = (ye - ys) * (n->x1 - n->x) - (xe - xs) * (n->y1 - n->y);
            COOR_TYPE numera = (xe - xs) * (n->y - ys) - (ye - ys) * (n->x - xs);
            COOR_TYPE numerb = (n->x1 - n->x) * (n->y - ys) - (n->y1 - n->y) * (n->x - xs);
            // �����Ƿ��غ�
            if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS)
            {
                return true;
            }
            // ���Ƿ�ƽ��
            if (fabs(denom) < EPS)
            {
                return false;
            }
            // �Ƿ������߶εĽ���
            mua = numera / denom;
            mub = numerb / denom;
            if (mua < 0 || mua > 1 || mub < 0 || mub > 1)
            {
                return true;
            }
            return true;
        }


        // ֱ����ֱ���ཻ(���ݲ�)
        bool LineLineIntersect(COOR_TYPE xs1, COOR_TYPE ys1, COOR_TYPE xe1, COOR_TYPE ye1,
            COOR_TYPE xs2, COOR_TYPE ys2, COOR_TYPE xe2, COOR_TYPE ye2, COOR_TYPE tol) const
        {
            COOR_TYPE mua, mub;

            double angle1 = atan2((ye1 - ys1), (xe1 - xs1));
            double angle2 = atan2((ye2 - ys2), (xe2 - xs2));
            if (angle1 < PI)
            {
                xs1 = xs1 + tol * cos(angle1 + PI);
                ys1 = ys1 + tol * sin(angle1 + PI);
            }
            else
            {
                xs1 = xs1 + tol * cos(angle1 - PI);
                ys1 = ys1 + tol * sin(angle1 - PI);
            }
            xe1 = xe1 + tol * cos(angle1);
            ye1 = ye1 + tol * sin(angle1);

            if (angle2 < PI)
            {
                xs2 = xs2 + tol * cos(angle2 + PI);
                ys2 = ys2 + tol * sin(angle2 + PI);
            }
            else
            {
                xs2 = xs2 + tol * cos(angle2 - PI);
                ys2 = ys2 + tol * sin(angle2 - PI);
            }
            xe2 = xe2 + tol * cos(angle2);
            ye2 = ye2 + tol * sin(angle2);


            COOR_TYPE denom = (ye2 - ys2) * (xe1 - xs1) - (xe2 - xs2) * (ye1 - ys1);
            COOR_TYPE numera = (xe2 - xs2) * (ys1 - ys2) - (ye2 - ys2) * (xs1 - xs2);
            COOR_TYPE numerb = (xe1 - xs1) * (ys1 - ys2) - (ye1 - ys1) * (xs1 - xs2);

            // �����Ƿ��غ�
            if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS)
            {
                return true;
            }

            // ���Ƿ�ƽ��
            if (fabs(denom) < EPS)
            {
                return false;
            }

            // �Ƿ������߶εĽ���
            mua = numera / denom;
            mub = numerb / denom;
            if (mua < 0 || mua > 1 || mub < 0 || mub > 1)
            {
                return false;
            }

            return true;
        }

        //// ֱ����ֱ���ཻ(���ݲ�)(xs2:���x ys2:���y xe2:�յ�x ye2:�յ�y tol:�ݲ�ֵ)
        //bool LineLineIntersect(const nodeType* n, COOR_TYPE xs2, COOR_TYPE ys2, COOR_TYPE xe2, COOR_TYPE ye2,
        //    COOR_TYPE tol) const
        //{
        //    COOR_TYPE mua, mub;

        //    double angle2 = atan2((ye2 - ys2), (xe2 - xs2));
        //    if (angle2 < PI)
        //    {
        //        xs2 = xs2 + tol * cos(angle2 + PI);
        //        ys2 = ys2 + tol * sin(angle2 + PI);
        //    }
        //    else
        //    {
        //        xs2 = xs2 + tol * cos(angle2 - PI);
        //        ys2 = ys2 + tol * sin(angle2 - PI);
        //    }
        //    xe2 = xe2 + tol * cos(angle2);
        //    ye2 = ye2 + tol * sin(angle2);

        //    COOR_TYPE denom = (ye2 - ys2) * (n->x1 - n->x) - (xe2 - xs2) * (n->y1 - n->y);
        //    COOR_TYPE numera = (xe2 - xs2) * (n->y - ys2) - (ye2 - ys2) * (n->x - xs2);
        //    COOR_TYPE numerb = (n->x1 - n->x) * (n->y - ys2) - (n->y1 - n->y) * (n->x - xs2);

        //    // �����Ƿ��غ�
        //    if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS)
        //    {
        //        return true;
        //    }

        //    // ���Ƿ�ƽ��
        //    if (fabs(denom) < EPS)
        //    {
        //        return false;
        //    }

        //    // �Ƿ������߶εĽ���
        //    mua = numera / denom;
        //    mub = numerb / denom;
        //    if (mua < 0 || mua > 1 || mub < 0 || mub > 1)
        //    {
        //        return true;
        //    }

        //    return true;
        //}

        // ֱ����ֱ���ཻ(���ݲ�)(xs2:���x ys2:���y xe2:�յ�x ye2:�յ�y tol:�ݲ�ֵ)
        bool LineLineIntersect(const nodeType* n, COOR_TYPE xs2, COOR_TYPE ys2, COOR_TYPE xe2, COOR_TYPE ye2,
            COOR_TYPE tol) const
        {
            COOR_TYPE mua, mub;

            COOR_TYPE xs1 = n->x;
            COOR_TYPE ys1 = n->y;
            COOR_TYPE xe1 = n->x1;
            COOR_TYPE ye1 = n->y1;
            double angle1 = atan2((ye1 - ys1), (xe1 - xs1));
            if (angle1 < PI)
            {
                xs1 = xs1 + tol * cos(angle1 + PI);
                ys1 = ys1 + tol * sin(angle1 + PI);
            }
            else
            {
                xs1 = xs1 + tol * cos(angle1 - PI);
                ys1 = ys1 + tol * sin(angle1 - PI);
            }
            xe1 = xe1 + tol * cos(angle1);
            ye1 = ye1 + tol * sin(angle1);

            double angle2 = atan2((ye2 - ys2), (xe2 - xs2));
            if (angle2 < PI)
            {
                xs2 = xs2 + tol * cos(angle2 + PI);
                ys2 = ys2 + tol * sin(angle2 + PI);
            }
            else
            {
                xs2 = xs2 + tol * cos(angle2 - PI);
                ys2 = ys2 + tol * sin(angle2 - PI);
            }
            xe2 = xe2 + tol * cos(angle2);
            ye2 = ye2 + tol * sin(angle2);

            COOR_TYPE denom = (ye2 - ys2) * (xe1 - xs1) - (xe2 - xs2) * (ye1 - ys1);
            COOR_TYPE numera = (xe2 - xs2) * (ys1 - ys2) - (ye2 - ys2) * (xs1 - xs2);
            COOR_TYPE numerb = (xe1 - xs1) * (ys1 - ys2) - (ye1 - ys1) * (xs1 - xs2);

            // �����Ƿ��غ�
            if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS)
            {
                return true;
            }

            // ���Ƿ�ƽ��
            if (fabs(denom) < EPS)
            {
                return false;
            }

            // �Ƿ������߶εĽ���
            mua = numera / denom;
            mub = numerb / denom;
            if (mua < 0 || mua > 1 || mub < 0 || mub > 1)
            {
                return true;
            }

            return true;
        }

        // �ж��������Ƿ����
        bool IsEqual(COOR_TYPE a, COOR_TYPE b, COOR_TYPE tol) const
        {
            return (fabs(a - b) < tol);
        }

        // �ж��������Ƿ��ཻ������ཻ���϶�����һ�����εĶ�������һ��������
        bool Intersect(const nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h) const
        {
            COOR_TYPE x0 = (n->x + n->x1) / 2;
            COOR_TYPE y0 = (n->y + n->y1) / 2;
            COOR_TYPE vx = x > x0 ? x - x0 : x0 - x;
            COOR_TYPE vy = y > y0 ? y - y0 : y0 - y;
			//acutPrintf(_T("Intersect %f,%f %f,%f %f,%f %f %f "), n->x, n->y,n->x1, n->y1,x,y,w,h);
            COOR_TYPE hx = n->x > x0 ? n->x - x0 : x0 - n->x;
            COOR_TYPE hy = n->y > y0 ? n->y - y0 : y0 - n->y;
			//acutPrintf(_T("Intersect %f %f %f %f "), vx, vy,hx, hy);
            COOR_TYPE ux = vx > hx ? vx - hx : 0;
            COOR_TYPE uy = vy > hy ? vy - hy : 0;
			//acutPrintf(_T("Intersect SS %f %f %f %f "), ux, uy,w, h);
			//acutPrintf(_T("��ֵ %f "), ux * ux + uy * uy-w * h);
            return ux * ux + uy * uy <= w * h;
			//return Intersect(n->x, n->y, n->w, n->h, x,y,w,h);
        }

        // �ж����������Ƿ��ཻ
        bool Intersect(COOR_TYPE x1, COOR_TYPE y1, COOR_TYPE w1, COOR_TYPE h1,
            COOR_TYPE x2, COOR_TYPE y2, COOR_TYPE w2, COOR_TYPE h2) const
        {
            COOR_TYPE x1_2 = x1 + w1;
            COOR_TYPE y1_2 = y1 + h1;
            COOR_TYPE x2_2 = x2 + w2;
            COOR_TYPE y2_2 = y2 + h2;
			//acutPrintf(_T("Intersect EX %f,%f %f,%f "), x1, y1,x2, y2);
            return (((x1 >= x2 && x1 < x2_2) || (x2 >= x1 && x2 <= x1_2)) &&
                ((y1 >= y2 && y1 < y2_2) || (y2 >= y1 && y2 <= y1_2))) ? true : false;
        }

        // �жϾ�����ֱ���Ƿ��ཻ
        bool RectLineIntersect(const nodeType* n, COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye) const
        {
            // ͨ�����ζ���
            if (xs > n->x && xs < n->x1 && ys > n->y && ys < n->y1)
            {
                return true;
            }
            if (xe > n->x && xe < n->x1 && ye > n->y && ye < n->y1)
            {
                return true;
            }
            AcGePoint3dArray points;
            points.append(AcGePoint3d(n->x, n->y, 0.0));
            points.append(AcGePoint3d(n->x, n->y1, 0.0));
            points.append(AcGePoint3d(n->x1, n->y1, 0.0));
            points.append(AcGePoint3d(n->x1, n->y, 0.0));
            // ͨ�����α�
            for (int i = 0; i < 4; ++i)
            {
                AcGePoint3d p1 = points[i];
                COOR_TYPE x1, y1, x2, y2;
                x1 = p1.x;
                y1 = p1.y;
                if (i == 3)
                {
                    x2 = points[0].x;
                    y2 = points[0].y;
                }
                else
                {
                    x2 = points[i + 1].x;
                    y2 = points[i + 1].y;
                }
                if (LineLineIntersect(x1, y1, x2, y2, xs, ys, xe, ye))
                {
                    return true;
                }
            }
            return false;
        }

        // �����Ƿ������
        bool ContainsVector(const nodeType* n, COOR_TYPE x, COOR_TYPE y) const
        {
            return (x >= n->x && y >= n->y && x <= n->x1 && y <= n->y1);
        }

        // �жϾ�����Բ�Ƿ��ཻ
        bool RectCircleIntersect(const nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE r) const
        {
            // x�����ң�y���ϵ������о��ε�����
            COOR_TYPE dist;
            if (n->x > n->x1)
            {
                dist = n->x1;
                n->x1 = n->x;
                n->x = dist;
            }
            if (n->y > n->y1)
            {
                dist = n->y1;
                n->y1 = n->y;
                n->y = dist;
            }
            // ���Բ���ʺ;��ε��������Ƿ��������߱�ʾ���������߿�����һ�����ֱ�ʾ
            dist = r / (COOR_TYPE)1.4142136;
            COOR_TYPE xSquare1 = x - dist;
            COOR_TYPE xSquare2 = x + dist;
            COOR_TYPE ySquare1 = y - dist;
            COOR_TYPE ySquare2 = y + dist;

            if (xSquare1 <= n->x1 && xSquare2 >= n->x && ySquare1 <= n->y && ySquare2 >= n->y1)
            {
                return true;
            }

            // �����ε�Բ�����Ƿ��о��εĶ���
            COOR_TYPE xRectangle = (n->x + n->x1) / 2;
            COOR_TYPE yRectangle = (n->y + n->y1) / 2;

            // ������Բ�����Ͻ�
            if (xRectangle <= x && yRectangle <= y &&
                sqrt((n->x1 - x) * (n->x1 - x) + (n->y1 - y) * (n->y1 - y)) <= r)
                return true;

            else if (xRectangle > x && yRectangle <= y &&
                sqrt((n->x - x) * (n->x - x) + (n->y1 - y) * (n->y1 - y)) <= r)
                return true;

            else if (xRectangle <= x && yRectangle > y &&
                sqrt((n->x1 - x) * (n->x1 - x) + (n->y - y) * (n->y - y)) <= r)
                return true;

            else if (sqrt((n->x - x) * (n->x - x) + (n->y - y) * (n->y - y)) <= r)
                return true;

            // ���Բ���ĸ������Ƿ���һ���ھ�����
            if ((x - r >= n->x && x - r <= n->x1 || x + r >= n->x && x + r <= n->x1) && y >= n->y && y <= n->y1 ||
                (y - r >= n->y && y - r >= n->y1 || y + r >= n->y && y + r <= n->y1) && x >= n->x && x <= n->x1)
                return true;
            
            return false;
        }
        
        // ����ָ���뾶�ڵĽڵ�
        void Search(const nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE radius, std::vector<const nodeType*>& result) const
        {
            if (n != NULL)
            {
                if (n->leaf)
                {
                    // �Ƿ��ڽڵ�ķ�Χ��
                    //if (InReach(n->x, n->y, x, y, radius))
                    if (Intersect(n->x, n->y, n->r, x, y, radius))
                    {
                        result.push_back(n);
                    }
                }
                else if (Intersect(n, x, y, radius) && n->node_size > 0)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (n->child[i])
                        {
                            Search(n->child[i], x, y, radius, result);
                        }
                    }
                }
            }
        }

        // ����ָ�������ڵĽڵ�(���x yֵ ��w ��h)
        void Search(const nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h, std::vector<const nodeType*>& result) const
        {
            if (n != NULL)
            {
                if (n->leaf)
                {
                    //acutPrintf(_T("n->leaf true"));
					// �ж����������Ƿ��ཻ
                    if (Intersect(n->x, n->y, n->w, n->h, x, y, w, h))
                    {
                        //acutPrintf(_T(" push_back  "));
                        result.push_back(n);
                    }
                }
                else if (Intersect(n, x, y, w, h) && n->node_size > 0)
                {
                    //acutPrintf(_T("n->leaf false"));
					for (int i = 0; i < 4; i++)
                    {
                        if (n->child[i])
                        {
                            Search(n->child[i], x, y, w, h, result);
                            //acutPrintf(_T("n->node Search  "));
                        }
                    }
                }
            }
        }

        // �����ֱཻ��(xs:���x ys:���y xe:�յ�x ye:�յ�y)
        void SearchLine(const nodeType* n, COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye,
            std::vector<const nodeType*>& result) const
        {
            if (n != NULL)
            {
                if (n->leaf)
                {
                    if (LineLineIntersect(n->xs, n->ys, n->xe, n->ye, xs, ys, xe, ye))
                    {
                        result.push_back(n);
                    }
                }
                else if (LineLineIntersect(n, xs, ys, xe, ye) && n->node_size > 0)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (n->child[i])
                        {
                            SearchLine(n->child[i], xs, ys, xe, ye, result);
                        }
                    }
                }
            }
        }

        // �����ֱཻ��(���ݲ�) (xs:���x ys:���y xe:�յ�x ye:�յ�y tol:�ݲ�ֵ)
        void SearchLine(const nodeType* n, COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye, COOR_TYPE tol,
            std::vector<const nodeType*>& result) const
        {
            if (n != NULL)
            {
                if (n->leaf)
                {
                    if (LineLineIntersect(n->xs, n->ys, n->xe, n->ye, xs, ys, xe, ye, tol))
                    {
                        result.push_back(n);
                    }
                }
                else if (LineLineIntersect(n, xs, ys, xe, ye, tol) && n->node_size > 0)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        if (n->child[i])
                        {
                            SearchLine(n->child[i], xs, ys, xe, ye, tol, result);
                        }
                    }
                }
            }
        }
        
        // �ж� x ֵ�Ƿ��� a ֵ�� b ֵ֮�� 
        bool Between(COOR_TYPE a, COOR_TYPE b, COOR_TYPE x)
        {
            //return a <= b ? (a <= x && x < b) : (a >= x && x > b);
            if ((x - b > -1.0e-7) && (x - b < 1.0e-7) || (x - a > -1.0e-7) && (x - a < 1.0e-7))
            {
                return true;
            }
            else {
                return a <= b ? (a <= x && x < b) : (a >= x && x > b);
            }
        }
        
        // �Ƿ��ڱ߽���
        bool Inbound(COOR_TYPE left, COOR_TYPE top, COOR_TYPE right, COOR_TYPE bottom, COOR_TYPE x, COOR_TYPE y)
        {
            return Between(left, right, x) && Between(top, bottom, y);
        }
        
        // �������Ƿ����
        bool Equal(COOR_TYPE a, COOR_TYPE b)
        {
            return a == b || ((std::fabs(a - b) < 1 - std::numeric_limits<COOR_TYPE>::epsilon())
                && (std::fabs(a - b) < std::numeric_limits<COOR_TYPE>::epsilon() * std::fabs(a + b)
                    || std::fabs(a - b) < (std::numeric_limits<COOR_TYPE>::min)()));
        }
        
        // ��ӽڵ�(2D��)
        bool AddNode(nodeType* n, COOR_TYPE x, COOR_TYPE y, DATA_TYPE data)
        {
            if (!Inbound(n->x, n->y, n->x1, n->y1, x, y))
            {
                return false;
            }
            COOR_TYPE mid_x = (n->x + n->x1) / 2;
            COOR_TYPE mid_y = (n->y + n->y1) / 2;
            COOR_TYPE left, top, right, bottom;
            int index = -1;
            if (Inbound(n->x, n->y, mid_x, mid_y, x, y))
            {
                left = n->x;
                top = n->y;
                right = mid_x;
                bottom = mid_y;
                index = 0;
            }
            else if (Inbound(mid_x, n->y, n->x1, mid_y, x, y))
            {
                left = mid_x;
                top = n->y;
                right = n->x1;
                bottom = mid_y;
                index = 1;
            }
            else if (Inbound(mid_x, mid_y, n->x1, n->y1, x, y))
            {
                left = mid_x;
                top = mid_y;
                right = n->x1;
                bottom = n->y1;
                index = 2;
            }
            else 
            {
                left = n->x;
                top = mid_y;
                right = mid_x;
                bottom = n->y1;
                index = 3;
            }
            if (Equal(left, right) || Equal(top, bottom))
            {
                return false;
            }
            if (n->child[index])
            {
                if (n->child[index]->leaf)
                {
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        return false;
                    }
                    nodeType* new_node = new nodeType(left, top, right, bottom);
                    if (AddNode(new_node, n->child[index]->x, n->child[index]->y, n->child[index]->data)
                        && AddNode(new_node, x, y, data))
                    {
                        ReleaseNode(n->child[index]);
                        n->child[index] = new_node;
                        n->node_size += 1;
                        return true;
                    }
                    else
                    {
                        ReleaseNode(new_node);
                        return false;
                    }
                }
                else
                {
                    if (AddNode(n->child[index], x, y, data))
                    {
                        n->node_size += 1;
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                n->child[index] = new nodeType(x, y, data);
                n->node_size += 1;
                return true;
            }
        }

        // ��ӽڵ�(Բ: x y �뾶r)
        bool AddNode(nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE r, DATA_TYPE data)
        {
            if (!Inbound(n->x, n->y, n->x1, n->y1, x, y))
            {
                return false;
            }
            COOR_TYPE mid_x = (n->x + n->x1) / 2.0;
            COOR_TYPE mid_y = (n->y + n->y1) / 2.0;
            COOR_TYPE left, top, right, bottom;
            int index = -1;
            if (Inbound(n->x, n->y, mid_x, mid_y, x, y))
            {
                left = n->x;
                top = n->y;
                right = mid_x;
                bottom = mid_y;
                index = 0;
            }
            else if (Inbound(mid_x, n->y, n->x1, mid_y, x, y))
            {
                left = mid_x;
                top = n->y;
                right = n->x1;
                bottom = mid_y;
                index = 1;
            }
            else if (Inbound(mid_x, mid_y, n->x1, n->y1, x, y))
            {
                left = mid_x;
                top = mid_y;
                right = n->x1;
                bottom = n->y1;
                index = 2;
            }
            else
            {
                left = n->x;
                top = mid_y;
                right = mid_x;
                bottom = n->y1;
                index = 3;
            }
            if (Equal(left, right) || Equal(top, bottom))
            {
                return false;
            }
            if (n->child[index])
            {
                if (n->child[index]->leaf)
                {
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        return false;
                    }
                    nodeType* new_node = new nodeType(left, top, right, bottom);
                    if (AddNode(new_node, n->child[index]->x, n->child[index]->y, n->child[index]->r,
                        n->child[index]->data) && AddNode(new_node, x, y, r, data))
                    {
                        ReleaseNode(n->child[index]);
                        n->child[index] = new_node;
                        n->node_size += 1;
                        return true;
                    }
                    else
                    {
                        ReleaseNode(new_node);
                        return false;
                    }
                }
                else
                {
                    if (AddNode(n->child[index], x, y, r, data))
                    {
                        n->node_size += 1;
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                n->child[index] = new nodeType(x, y, r, data);
                n->node_size += 1;
                return true;
            }
        }

        // ��ӽڵ�(����: x y ���w �߶�h)
        bool AddNode(nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h, DATA_TYPE data)
        {
            int num = 0;
            if (!Inbound(n->x, n->y, n->x1, n->y1, x, y))
            {
                //acutPrintf(_T("\n!Inbound(n->x, n->y, n->x1, n->y1, x, y) %f  %f %f  %f x %f y %f"), n->x, n->y, n->x1, n->y1, x,y);
                return false;
            }
            COOR_TYPE mid_x = (n->x + n->x1) / 2;
            COOR_TYPE mid_y = (n->y + n->y1) / 2;
            COOR_TYPE left, top, right, bottom;
            int index = -1;
            if (Inbound(n->x, n->y, mid_x, mid_y, x, y))
            {
                left = n->x;
                top = n->y;
                right = mid_x;
                bottom = mid_y;
                index = 0;
            }
            else if (Inbound(mid_x, n->y, n->x1, mid_y, x, y))
            {
                left = mid_x;
                top = n->y;
                right = n->x1;
                bottom = mid_y;
                index = 1;
            }
            else if (Inbound(mid_x, mid_y, n->x1, n->y1, x, y))
            {
                left = mid_x;
                top = mid_y;
                right = n->x1;
                bottom = n->y1;
                index = 2;
            }
            else
            {
                left = n->x;
                top = mid_y;
                right = mid_x;
                bottom = n->y1;
                index = 3;
            }
            if (Equal(left, right) || Equal(top, bottom))
            {
                //acutPrintf(_T("\nEqual(left, right) || Equal(top, bottom)"));
                return false;
            }
            if (n->child[index])
            {
                if (n->child[index]->leaf)
                {
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        return false;
                    }
                    nodeType* new_node = new nodeType(left, top, right, bottom);
                    if (AddNode(new_node, n->child[index]->x, n->child[index]->y,
                        n->child[index]->w, n->child[index]->h,
                        n->child[index]->data) &&
                        AddNode(new_node, x, y, w, h, data))
                    {
                        ReleaseNode(n->child[index]);
                        n->child[index] = new_node;
                        n->node_size += 1;
                        num++;
                        //acutPrintf(_T("index %d num %d "), index, num);
                        return true;
                    }
                    else
                    {
                        //acutPrintf(_T("ReleaseNode false"));
                        ReleaseNode(new_node);
                        return false;
                    }
                }
                else
                {
                    if (AddNode(n->child[index], x, y, w, h, data))
                    {
                        n->node_size += 1;
                        num++;
                        //acutPrintf(_T("index %d num %d "), index, num);
                        return true;
                    }
                    else
                    {
                        //acutPrintf(_T("false"));
                        return false;
                    }
                }
            }
            else
            {
                n->child[index] = new nodeType(x, y, w, h, data);
                n->node_size += 1;
                num++;
                //acutPrintf(_T("index %d num %d "), index, num);
                return true;
            }
        }
        
        // ��ӽڵ�(ֱ��)(xs:���x ys:���y xe:�յ�x ye:�յ�y tol:�ݲ� dtat:�Զ�������)
        bool AddNode(nodeType* n, COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye, COOR_TYPE tol, DATA_TYPE data)
        {
            COOR_TYPE x = (xs + xe) * 0.5;
            COOR_TYPE y = (ys + ye) * 0.5;

            if (!Inbound(n->x, n->y, n->x1, n->y1, x, y))
            {
                return false;
            }
            COOR_TYPE mid_x = (n->x + n->x1) / 2;
            COOR_TYPE mid_y = (n->y + n->y1) / 2;
            COOR_TYPE left, top, right, bottom;
            int index = -1;
            if (Inbound(n->x, n->y, mid_x, mid_y, x, y))
            {
                left = n->x;
                top = n->y;
                right = mid_x;
                bottom = mid_y;

                index = 0;
            }
            else if (Inbound(mid_x, n->y, n->x1, mid_y, x, y))
            {
                left = mid_x;
                top = n->y;
                right = n->x1;
                bottom = mid_y;

                index = 1;
            }
            else if (Inbound(mid_x, mid_y, n->x1, n->y1, x, y))
            {
                left = mid_x;
                top = mid_y;
                right = n->x1;
                bottom = n->y1;

                index = 2;
            }
            else
            {
                left = n->x;
                top = mid_y;
                right = mid_x;
                bottom = n->y1;

                index = 3;
            }
            if (Equal(left, right) || Equal(top, bottom))
            {
                return false;
            }
            if (n->child[index])
            {
                if (n->child[index]->leaf)
                {
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        return false;
                    }
                    nodeType* new_node = new nodeType(left, top, right, bottom);
                    if (AddNode(new_node, n->child[index]->xs, n->child[index]->ys,
                        n->child[index]->xe, n->child[index]->ye,
                        n->child[index]->tol, n->child[index]->data) &&
                        AddNode(new_node, xs, ys, xe, ye, tol, data))
                    {
                        ReleaseNode(n->child[index]);
                        n->child[index] = new_node;
                        n->node_size += 1;
                        return true;
                    }
                    else
                    {
                        ReleaseNode(new_node);
                        return false;
                    }
                }
                else
                {
                    if (AddNode(n->child[index], xs, ys, xe, ye, tol, data))
                    {
                        n->node_size += 1;
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                n->child[index] = new nodeType(xs, ys, xe, ye, tol, data);
                n->node_size += 1;
                return true;
            }
        }

        // ɾ���ڵ�(2D�� x y)
        bool DeleteNode(nodeType* n, COOR_TYPE x, COOR_TYPE y, DATA_TYPE data)
        {
            if (!Inbound(n->x, n->y, n->x1, n->y1, x, y))
            {
                return false;
            }
            COOR_TYPE mid_x = (n->x + n->x1) / 2;
            COOR_TYPE mid_y = (n->y + n->y1) / 2;
            COOR_TYPE left, top, right, bottom;
            int index = -1;
            if (Inbound(n->x, n->y, mid_x, mid_y, x, y))
            {
                left = n->x;
                top = n->y;
                right = mid_x;
                bottom = mid_y;
                index = 0;
            }
            else if (Inbound(mid_x, n->y, n->x1, mid_y, x, y))
            {
                left = mid_x;
                top = n->y;
                right = n->x1;
                bottom = mid_y;
                index = 1;
            }
            else if (Inbound(mid_x, mid_y, n->x1, n->y1, x, y))
            {
                left = mid_x;
                top = mid_y;
                right = n->x1;
                bottom = n->y1;
                index = 2;
            }
            else
            {
                left = n->x;
                top = mid_y;
                right = mid_x;
                bottom = n->y1;
                index = 3;
            }
            if (Equal(left, right) || Equal(top, bottom))
            {
                return false;
            }
            if (n->child[index])
            {
                if (n->child[index]->leaf)
                {
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        return false;
                    }
                    nodeType* new_node = new nodeType(left, top, right, bottom);
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        ReleaseNode(n->child[index]);
                        n->child[index] = new_node;
                        n->node_size -= 1;
                        return true;
                    }
                    else
                    {
                        ReleaseNode(new_node);
                        return false;
                    }
                }
                else
                {
                    if (DeleteNode(n->child[index], x, y, data))
                    {
                        n->node_size -= 1;
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                n->child[index] = new nodeType(x, y, data);
                n->node_size -= 1;
                return true;
            }
        }

        // ɾ���ڵ�(Բ x y �뾶r)
        bool DeleteNode(nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE r, DATA_TYPE data)
        {
            if (!Inbound(n->x, n->y, n->x1, n->y1, x, y))
            {
                return false;
            }
            COOR_TYPE mid_x = (n->x + n->x1) / 2.0;
            COOR_TYPE mid_y = (n->y + n->y1) / 2.0;
            COOR_TYPE left, top, right, bottom;
            int index = -1;
            if (Inbound(n->x, n->y, mid_x, mid_y, x, y))
            {
                left = n->x;
                top = n->y;
                right = mid_x;
                bottom = mid_y;
                index = 0;
            }
            else if (Inbound(mid_x, n->y, n->x1, mid_y, x, y))
            {
                left = mid_x;
                top = n->y;
                right = n->x1;
                bottom = mid_y;
                index = 1;
            }
            else if (Inbound(mid_x, mid_y, n->x1, n->y1, x, y))
            {
                left = mid_x;
                top = mid_y;
                right = n->x1;
                bottom = n->y1;
                index = 2;
            }
            else
            {
                left = n->x;
                top = mid_y;
                right = mid_x;
                bottom = n->y1;
                index = 3;
            }
            if (Equal(left, right) || Equal(top, bottom))
            {
                return false;
            }
            if (n->child[index])
            {
                if (n->child[index]->leaf)
                {
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        return false;
                    }
                    nodeType* new_node = new nodeType(left, top, right, bottom);
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y) &&
                        Equal(n->child[index]->r, r))
                    {
                        ReleaseNode(n->child[index]);
                        n->child[index] = new_node;
                        n->node_size -= 1;
                        return true;
                    }
                    else
                    {
                        ReleaseNode(new_node);
                        return false;
                    }
                }
                else
                {
                    if (DeleteNode(n->child[index], x, y, r, data))
                    {
                        n->node_size -= 1;
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                n->child[index] = new nodeType(x, y, r, data);
                n->node_size -= 1;
                return true;
            }
        }

        // ɾ���ڵ�(���� x y ��w ��h)
        bool DeleteNode(nodeType* n, COOR_TYPE x, COOR_TYPE y, COOR_TYPE w, COOR_TYPE h, DATA_TYPE data)
        {
            if (!Inbound(n->x, n->y, n->x1, n->y1, x, y))
            {
                return false;
            }
            COOR_TYPE mid_x = (n->x + n->x1) / 2;
            COOR_TYPE mid_y = (n->y + n->y1) / 2;
            COOR_TYPE left, top, right, bottom;
            int index = -1;
            if (Inbound(n->x, n->y, mid_x, mid_y, x, y))
            {
                left = n->x;
                top = n->y;
                right = mid_x;
                bottom = mid_y;
                index = 0;
            }
            else if (Inbound(mid_x, n->y, n->x1, mid_y, x, y))
            {
                left = mid_x;
                top = n->y;
                right = n->x1;
                bottom = mid_y;
                index = 1;
            }
            else if (Inbound(mid_x, mid_y, n->x1, n->y1, x, y))
            {
                left = mid_x;
                top = mid_y;
                right = n->x1;
                bottom = n->y1;
                index = 2;
            }
            else
            {
                left = n->x;
                top = mid_y;
                right = mid_x;
                bottom = n->y1;
                index = 3;
            }
            if (Equal(left, right) || Equal(top, bottom))
            {
                return false;
            }
            if (n->child[index])
            {
                if (n->child[index]->leaf)
                {
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        return false;
                    }
                    nodeType* new_node = new nodeType(left, top, right, bottom);
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y) &&
                        Equal(n->child[index]->w, w) && Equal(n->child[index]->h, h))
                    {
                        ReleaseNode(n->child[index]);
                        n->child[index] = new_node;
                        n->node_size -= 1;
                        return true;
                    }
                    else
                    {
                        ReleaseNode(new_node);
                        return false;
                    }
                }
                else
                {
                    if (DeleteNode(n->child[index], x, y, w, h, data))
                    {
                        n->node_size -= 1;
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                n->child[index] = new nodeType(x, y, w, h, data);
                n->node_size -= 1;
                return true;
            }
        }

        // ɾ���ڵ�(ֱ�� xs:���x ys:���y  xe:�յ�x ye:�յ�y tol:�ݲ� data:�Զ�������)
        bool DeleteNode(nodeType* n, COOR_TYPE xs, COOR_TYPE ys, COOR_TYPE xe, COOR_TYPE ye, COOR_TYPE tol, DATA_TYPE data)
        {
            COOR_TYPE x = (xs + xe) * 0.5;
            COOR_TYPE y = (ys + ye) * 0.5;

            if (!Inbound(n->x, n->y, n->x1, n->y1, x, y))
            {
                return false;
            }
            COOR_TYPE mid_x = (n->x + n->x1) / 2;
            COOR_TYPE mid_y = (n->y + n->y1) / 2;
            COOR_TYPE left, top, right, bottom;
            int index = -1;
            if (Inbound(n->x, n->y, mid_x, mid_y, x, y))
            {
                left = n->x;
                top = n->y;
                right = mid_x;
                bottom = mid_y;

                index = 0;
            }
            else if (Inbound(mid_x, n->y, n->x1, mid_y, x, y))
            {
                left = mid_x;
                top = n->y;
                right = n->x1;
                bottom = mid_y;

                index = 1;
            }
            else if (Inbound(mid_x, mid_y, n->x1, n->y1, x, y))
            {
                left = mid_x;
                top = mid_y;
                right = n->x1;
                bottom = n->y1;

                index = 2;
            }
            else
            {
                left = n->x;
                top = mid_y;
                right = mid_x;
                bottom = n->y1;

                index = 3;
            }
            if (Equal(left, right) || Equal(top, bottom))
            {
                return false;
            }
            if (n->child[index])
            {
                if (n->child[index]->leaf)
                {
                    if (Equal(n->child[index]->x, x) && Equal(n->child[index]->y, y))
                    {
                        return false;
                    }
                    nodeType* new_node = new nodeType(left, top, right, bottom);
                    if (Equal(n->child[index]->xs, xs) && Equal(n->child[index]->ys, ys) &&
                        Equal(n->child[index]->xe, xe) && Equal(n->child[index]->ye, ye))
                    {
                        ReleaseNode(n->child[index]);
                        n->child[index] = new_node;
                        n->node_size -= 1;
                        return true;
                    }
                    else
                    {
                        ReleaseNode(new_node);
                        return false;
                    }
                }
                else
                {
                    if (DeleteNode(n->child[index], xs, ys, xe, ye, tol, data))
                    {
                        n->node_size -= 1;
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
            else
            {
                n->node_size -= 1;
                return true;
            }
        }

        // �ͷŽڵ�
        void ReleaseNode(nodeType* node)
        {
            if (node)
            {
                if (!node->leaf)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        ReleaseNode(node->child[i]);
                    }
                }
                delete node;
            }
        }
    };
}

#endif //__G_QTREE__
