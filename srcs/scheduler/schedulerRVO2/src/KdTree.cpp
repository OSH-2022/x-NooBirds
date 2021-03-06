/*
 * KdTree.cpp
 * RVO2 Library
 *
 * Copyright (c) 2008-2010 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the Office of Technology Development at the University
 * of North Carolina at Chapel Hill <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "KdTree.h"

#include "Agent.h"
#include "RVOSimulator.h"
#include "Obstacle.h"

namespace RVO {
	KdTree::KdTree(RVOSimulator *sim) : obstacleTree_(NULL), sim_(sim) { }

	KdTree::~KdTree()
	{
		deleteObstacleTree(obstacleTree_);
	}

	void KdTree::buildAgentTree()
	{
		if (agentNos_.size() < sim_->agents_.size()) {
			for (size_t i = agentNos_.size(); i < sim_->agents_.size(); ++i) {
				agentNos_.push_back(i);
			}

			agentTree_.resize(2 * agentNos_.size() - 1);
		}

		if (!agentNos_.empty()) {
			buildAgentTreeRecursive(0, agentNos_.size(), 0);
		}
	}

	void KdTree::buildAgentTreeRecursive(size_t begin, size_t end, size_t node)
	{
		agentTree_[node].begin = begin;
		agentTree_[node].end = end;
		agentTree_[node].minX = agentTree_[node].maxX = sim_->agents_[agentNos_[begin]]->position_.x();
		agentTree_[node].minY = agentTree_[node].maxY = sim_->agents_[agentNos_[begin]]->position_.y();

		for (size_t i = begin + 1; i < end; ++i) {
			agentTree_[node].maxX = std::max(agentTree_[node].maxX, sim_->agents_[agentNos_[i]]->position_.x());
			agentTree_[node].minX = std::min(agentTree_[node].minX, sim_->agents_[agentNos_[i]]->position_.x());
			agentTree_[node].maxY = std::max(agentTree_[node].maxY, sim_->agents_[agentNos_[i]]->position_.y());
			agentTree_[node].minY = std::min(agentTree_[node].minY, sim_->agents_[agentNos_[i]]->position_.y());
		}

		if (end - begin > MAX_LEAF_SIZE) {
			/* No leaf node. */
			const bool isVertical = (agentTree_[node].maxX - agentTree_[node].minX > agentTree_[node].maxY - agentTree_[node].minY);
			const float splitValue = (isVertical ? 0.5f * (agentTree_[node].maxX + agentTree_[node].minX) : 0.5f * (agentTree_[node].maxY + agentTree_[node].minY));

			size_t left = begin;
			size_t right = end;

			while (left < right) {
				while (left < right && (isVertical ? sim_->agents_[agentNos_[left]]->position_.x() : sim_->agents_[agentNos_[left]]->position_.y()) < splitValue) {
					++left;
				}

				while (right > left && (isVertical ? sim_->agents_[agentNos_[right - 1]]->position_.x() : sim_->agents_[agentNos_[right - 1]]->position_.y()) >= splitValue) {
					--right;
				}

				if (left < right) {
					std::swap(agentNos_[left], agentNos_[right - 1]);
					++left;
					--right;
				}
			}

			if (left == begin) {
				++left;
				++right;
			}

			agentTree_[node].left = node + 1;
			agentTree_[node].right = node + 2 * (left - begin);

			buildAgentTreeRecursive(begin, left, agentTree_[node].left);
			buildAgentTreeRecursive(left, end, agentTree_[node].right);
		}
	}

	void KdTree::buildObstacleTree()
	{
		deleteObstacleTree(obstacleTree_);

		std::vector<size_t> obstacles(sim_->obstacles_.size());

		for (size_t i = 0; i < sim_->obstacles_.size(); ++i) {
			obstacles[i] = i;
		}

		obstacleTree_ = buildObstacleTreeRecursive(obstacles);
	}


	KdTree::ObstacleTreeNode *KdTree::buildObstacleTreeRecursive(const std::vector<size_t> &obstacleNos)
	{
		if (obstacleNos.empty()) {
			return NULL;
		}
		else {
			ObstacleTreeNode *const node = new ObstacleTreeNode;

			size_t optimalSplit = 0;
			size_t minLeft = obstacleNos.size();
			size_t minRight = obstacleNos.size();

			for (size_t i = 0; i < obstacleNos.size(); ++i) {
				size_t leftSize = 0;
				size_t rightSize = 0;

				const Obstacle *const obstacleI1 = sim_->obstacles_[obstacleNos[i]];
				const Obstacle *const obstacleI2 = sim_->obstacles_[obstacleI1->nextObstacle_];

				/* Compute optimal split node. */
				for (size_t j = 0; j < obstacleNos.size(); ++j) {
					if (i == j) {
						continue;
					}

					const Obstacle *const obstacleJ1 = sim_->obstacles_[obstacleNos[j]];
					const Obstacle *const obstacleJ2 = sim_->obstacles_[obstacleJ1->nextObstacle_];

					const float j1LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
					const float j2LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

					if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
						++leftSize;
					}
					else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
						++rightSize;
					}
					else {
						++leftSize;
						++rightSize;
					}

					if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) >= std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) {
						break;
					}
				}

				if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) < std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) {
					minLeft = leftSize;
					minRight = rightSize;
					optimalSplit = i;
				}
			}

			/* Build split node. */
			std::vector<size_t> leftObstacles(minLeft);
			std::vector<size_t> rightObstacles(minRight);

			size_t leftCounter = 0;
			size_t rightCounter = 0;
			const size_t i = optimalSplit;

			const Obstacle *const obstacleI1 = sim_->obstacles_[obstacleNos[i]];
			const Obstacle *const obstacleI2 = sim_->obstacles_[obstacleI1->nextObstacle_];

			for (size_t j = 0; j < obstacleNos.size(); ++j) {
				if (i == j) {
					continue;
				}

				Obstacle *const obstacleJ1 = sim_->obstacles_[obstacleNos[j]];
				Obstacle *const obstacleJ2 = sim_->obstacles_[obstacleJ1->nextObstacle_];

				const float j1LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
				const float j2LeftOfI = leftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

				if (j1LeftOfI >= -RVO_EPSILON && j2LeftOfI >= -RVO_EPSILON) {
					leftObstacles[leftCounter++] = obstacleNos[j];
				}
				else if (j1LeftOfI <= RVO_EPSILON && j2LeftOfI <= RVO_EPSILON) {
					rightObstacles[rightCounter++] = obstacleNos[j];
				}
				else {
					/* Split obstacle j. */
					const float t = det(obstacleI2->point_ - obstacleI1->point_, obstacleJ1->point_ - obstacleI1->point_) / det(obstacleI2->point_ - obstacleI1->point_, obstacleJ1->point_ - obstacleJ2->point_);

					const Vector2 splitpoint = obstacleJ1->point_ + t * (obstacleJ2->point_ - obstacleJ1->point_);

					Obstacle *const newObstacle = new Obstacle();
					newObstacle->point_ = splitpoint;
					newObstacle->prevObstacle_ = obstacleJ2->prevObstacle_;
					newObstacle->nextObstacle_ = obstacleJ1->nextObstacle_;
					newObstacle->isConvex_ = true;

					sim_->obstacles_.push_back(newObstacle);
					const size_t newObstacleNo = sim_->obstacles_.size() - 1;

					obstacleJ1->nextObstacle_ = newObstacleNo;
					obstacleJ2->prevObstacle_ = newObstacleNo;

					if (j1LeftOfI > 0.0f) {
						leftObstacles[leftCounter++] = obstacleNos[j];
						rightObstacles[rightCounter++] = newObstacleNo;
					}
					else {
						rightObstacles[rightCounter++] = obstacleNos[j];
						leftObstacles[leftCounter++] = newObstacleNo;
					}
				}
			}

			node->obstacleNo = obstacleNos[i];
			node->left = buildObstacleTreeRecursive(leftObstacles);
			node->right = buildObstacleTreeRecursive(rightObstacles);
			return node;
		}
	}

	void KdTree::computeAgentNeighbors(Agent *agent, float &rangeSq) const
	{
		queryAgentTreeRecursive(agent, rangeSq, 0);
	}

	void KdTree::computeObstacleNeighbors(Agent *agent, float rangeSq) const
	{
		queryObstacleTreeRecursive(agent, rangeSq, obstacleTree_);
	}

	void KdTree::deleteObstacleTree(ObstacleTreeNode *node)
	{
		if (node != NULL) {
			deleteObstacleTree(node->left);
			deleteObstacleTree(node->right);
			delete node;
		}
	}

	void KdTree::queryAgentTreeRecursive(Agent *agent, float &rangeSq, size_t node) const
	{
		if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE) {
			for (size_t i = agentTree_[node].begin; i < agentTree_[node].end; ++i) {
				agent->insertAgentNeighbor(agentNos_[i], rangeSq);
			}
		}
		else {
			const float distSqLeft = sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].left].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].left].maxY));

			const float distSqRight = sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minX - agent->position_.x())) + sqr(std::max(0.0f, agent->position_.x() - agentTree_[agentTree_[node].right].maxX)) + sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minY - agent->position_.y())) + sqr(std::max(0.0f, agent->position_.y() - agentTree_[agentTree_[node].right].maxY));

			if (distSqLeft < distSqRight) {
				if (distSqLeft < rangeSq) {
					queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);

					if (distSqRight < rangeSq) {
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);
					}
				}
			}
			else {
				if (distSqRight < rangeSq) {
					queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].right);

					if (distSqLeft < rangeSq) {
						queryAgentTreeRecursive(agent, rangeSq, agentTree_[node].left);
					}
				}
			}

		}
	}

	void KdTree::queryObstacleTreeRecursive(Agent *agent, float rangeSq, const ObstacleTreeNode *node) const
	{
		if (node == NULL) {
			return;
		}
		else {
			const Obstacle *const obstacle1 = sim_->obstacles_[node->obstacleNo];
			const Obstacle *const obstacle2 = sim_->obstacles_[obstacle1->nextObstacle_];

			const float agentLeftOfLine = leftOf(obstacle1->point_, obstacle2->point_, agent->position_);

			queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0.0f ? node->left : node->right));

			const float distSqLine = sqr(agentLeftOfLine) / absSq(obstacle2->point_ - obstacle1->point_);

			if (distSqLine < rangeSq) {
				if (agentLeftOfLine < 0.0f) {
					/*
					 * Try obstacle at this node only if agent is on right side of
					 * obstacle (and can see obstacle).
					 */
					agent->insertObstacleNeighbor(node->obstacleNo, rangeSq);
				}

				/* Try other side of line. */
				queryObstacleTreeRecursive(agent, rangeSq, (agentLeftOfLine >= 0.0f ? node->right : node->left));

			}
		}
	}

	bool KdTree::queryVisibility(const Vector2 &q1, const Vector2 &q2, float radius) const
	{
		return queryVisibilityRecursive(q1, q2, radius, obstacleTree_);
	}

	bool KdTree::queryVisibilityRecursive(const Vector2 &q1, const Vector2 &q2, float radius, const ObstacleTreeNode *node) const
	{
		if (node == NULL) {
			return true;
		}
		else {
			const Obstacle *const obstacle1 = sim_->obstacles_[node->obstacleNo];
			const Obstacle *const obstacle2 = sim_->obstacles_[obstacle1->nextObstacle_];

			const float q1LeftOfI = leftOf(obstacle1->point_, obstacle2->point_, q1);
			const float q2LeftOfI = leftOf(obstacle1->point_, obstacle2->point_, q2);
			const float invLengthI = 1.0f / absSq(obstacle2->point_ - obstacle1->point_);

			if (q1LeftOfI >= 0.0f && q2LeftOfI >= 0.0f) {
				return queryVisibilityRecursive(q1, q2, radius, node->left) && ((sqr(q1LeftOfI) * invLengthI >= sqr(radius) && sqr(q2LeftOfI) * invLengthI >= sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node->right));
			}
			else if (q1LeftOfI <= 0.0f && q2LeftOfI <= 0.0f) {
				return queryVisibilityRecursive(q1, q2, radius, node->right) && ((sqr(q1LeftOfI) * invLengthI >= sqr(radius) && sqr(q2LeftOfI) * invLengthI >= sqr(radius)) || queryVisibilityRecursive(q1, q2, radius, node->left));
			}
			else if (q1LeftOfI >= 0.0f && q2LeftOfI <= 0.0f) {
				/* One can see through obstacle from left to right. */
				return queryVisibilityRecursive(q1, q2, radius, node->left) && queryVisibilityRecursive(q1, q2, radius, node->right);
			}
			else {
				const float point1LeftOfQ = leftOf(q1, q2, obstacle1->point_);
				const float point2LeftOfQ = leftOf(q1, q2, obstacle2->point_);
				const float invLengthQ = 1.0f / absSq(q2 - q1);

				return (point1LeftOfQ * point2LeftOfQ >= 0.0f && sqr(point1LeftOfQ) * invLengthQ > sqr(radius) && sqr(point2LeftOfQ) * invLengthQ > sqr(radius) && queryVisibilityRecursive(q1, q2, radius, node->left) && queryVisibilityRecursive(q1, q2, radius, node->right));
			}
		}
	}
}
