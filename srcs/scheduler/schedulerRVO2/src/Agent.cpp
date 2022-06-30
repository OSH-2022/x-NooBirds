/*
 * Agent.cpp
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

#include "Agent.h"

#include "KdTree.h"
#include "Obstacle.h"

namespace RVO {
	Agent::Agent(RVOSimulator *sim) : maxNeighbors_(0), maxSpeed_(0.0f), neighborDist_(0.0f), radius_(0.0f), sim_(sim), timeHorizon_(0.0f), timeHorizonObst_(0.0f) { }

	Agent::Agent(RVOSimulator *sim, const Vector2 &position) : maxNeighbors_(sim->defaultAgent_->maxNeighbors_), maxSpeed_(sim->defaultAgent_->maxSpeed_), neighborDist_(sim->defaultAgent_->neighborDist_), newVelocity_(sim->defaultAgent_->velocity_), position_(position), radius_(sim->defaultAgent_->radius_), sim_(sim), timeHorizon_(sim->defaultAgent_->timeHorizon_), timeHorizonObst_(sim->defaultAgent_->timeHorizonObst_), velocity_(sim->defaultAgent_->velocity_) { }

	Agent::Agent(RVOSimulator *sim, const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, const Vector2 &velocity, float maxSpeed) : maxNeighbors_(maxNeighbors), maxSpeed_(maxSpeed), neighborDist_(neighborDist), newVelocity_(velocity), position_(position), radius_(radius), sim_(sim), timeHorizon_(timeHorizon), timeHorizonObst_(timeHorizonObst), velocity_(velocity) { }

	void Agent::computeNeighbors()
	{
		obstacleNeighbors_.clear();
		float rangeSq = sqr(timeHorizonObst_ * maxSpeed_ + radius_);
		sim_->kdTree_->computeObstacleNeighbors(this, rangeSq);

		agentNeighbors_.clear();

		if (maxNeighbors_ > 0) {
			rangeSq = sqr(neighborDist_);
			sim_->kdTree_->computeAgentNeighbors(this, rangeSq);
		}
	}

	/* Search for the best new velocity. */
	void Agent::computeNewVelocity()
	{
		orcaLines_.clear();

		/* Create obstacle ORCA lines. */
		for (size_t i = 0; i < obstacleNeighbors_.size(); ++i) {
			float invTimeHorizonObst = 1.0f / timeHorizonObst_;

			const Obstacle *obstacle1 = sim_->obstacles_[obstacleNeighbors_[i].second];
			const Obstacle *obstacle2 = sim_->obstacles_[obstacle1->nextObstacle_];

			const Vector2 relativePosition1 = obstacle1->point_ - position_;
			const Vector2 relativePosition2 = obstacle2->point_ - position_;
			const Vector2 obstacleVector = obstacle2->point_ - obstacle1->point_;

			/*
			 * Check if velocity obstacle of obstacle is already taken care of by
			 * previously constructed obstacle ORCA lines.
			 */
			bool alreadyCovered = false;

			for (size_t j = 0; j < orcaLines_.size(); ++j) {
				if (det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVO_EPSILON && det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >=  -RVO_EPSILON) {
					alreadyCovered = true;
					break;
				}
			}

			if (alreadyCovered) {
				continue;
			}

			/*
			 * Compute legs. When obliquely viewed, both legs can come from a single
			 * vertex. Legs can never point into neighboring edge when convex vertex,
			 * take cutoff-line of neighboring edge instead. If velocity projected on
			 * "foreign" leg, no constraint is added. Legs extend cut-off line when
			 * nonconvex vertex.
			 */
			const float distSq1 = absSq(relativePosition1);
			const float distSq2 = absSq(relativePosition2);

			const float radiusSq = sqr(radius_);

			Vector2 leftLegDirection, rightLegDirection;

			const float s = (-relativePosition1 * obstacleVector) / absSq(obstacleVector);
			const float distSqLine = absSq(-relativePosition1 - s * obstacleVector);

			if (s < 0.0f && distSq1 <= radiusSq) {
				/* Collision with left vertex. */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				invTimeHorizonObst = 1.0f / sim_->timeStep_;
				obstacle2 = obstacle1;
				leftLegDirection = normalize(Vector2(-relativePosition1.y(), relativePosition1.x()));
				rightLegDirection = -leftLegDirection;
			}
			else if (s > 1.0f && distSq2 <= radiusSq) {
				/* Collision with right vertex. */
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				invTimeHorizonObst = 1.0f / sim_->timeStep_;
				obstacle1 = obstacle2;
				leftLegDirection = normalize(Vector2(-relativePosition2.y(), relativePosition2.x()));
				rightLegDirection = -leftLegDirection;
			}
			else if (s >= 0.0f && s < 1.0f && distSqLine <= radiusSq) {
				/* Collision with obstacle segment. */
				invTimeHorizonObst = 1.0f / sim_->timeStep_;
				leftLegDirection = -normalize(obstacleVector);
				rightLegDirection = -leftLegDirection;
			}
			else if (s < 0.0f && distSqLine <= radiusSq) {
				/*
				 * No collision, but obstacle viewed obliquely so that left vertex
				 * defines velocity obstacle.
				 */
				if (!obstacle1->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle2 = obstacle1;

				const float leg1 = std::sqrt(distSq1 - radiusSq);

				leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				rightLegDirection = Vector2(relativePosition1.x() * leg1 + relativePosition1.y() * radius_, -relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
			}
			else if (s > 1.0f && distSqLine <= radiusSq) {
				/*
				 * No collision, but obstacle viewed obliquely so that
				 * right vertex defines velocity obstacle.
				 */
				if (!obstacle2->isConvex_) {
					/* Ignore obstacle. */
					continue;
				}

				obstacle1 = obstacle2;

				const float leg2 = std::sqrt(distSq2 - radiusSq);

				leftLegDirection = Vector2(relativePosition2.x() * leg2 - relativePosition2.y() * radius_, relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
			}
			else {
				/* Usual situation. */
				if (obstacle1->isConvex_) {
					const float leg1 = std::sqrt(distSq1 - radiusSq);
					leftLegDirection = Vector2(relativePosition1.x() * leg1 - relativePosition1.y() * radius_, relativePosition1.x() * radius_ + relativePosition1.y() * leg1) / distSq1;
				}
				else {
					/* Left vertex non-convex; left leg extends cut-off line. */
					leftLegDirection = -normalize(obstacleVector);
				}

				if (obstacle2->isConvex_) {
					const float leg2 = std::sqrt(distSq2 - radiusSq);
					rightLegDirection = Vector2(relativePosition2.x() * leg2 + relativePosition2.y() * radius_, -relativePosition2.x() * radius_ + relativePosition2.y() * leg2) / distSq2;
				}
				else {
					/* Right vertex non-convex; right leg extends cut-off line. */
					rightLegDirection = (obstacle1->isConvex_ ? normalize(obstacleVector) : -leftLegDirection);
				}
			}

			const Obstacle *const leftNeighbor = sim_->obstacles_[obstacle1->prevObstacle_];
			const Obstacle *const rightNeighbor = sim_->obstacles_[obstacle2->nextObstacle_];

			bool isLeftLegForeign = false;
			bool isRightLegForeign = false;

			if (obstacle1->isConvex_ && det(leftLegDirection, leftNeighbor->point_ - obstacle1->point_) > 0.0f) {
				/* Leg points into obstacle. */
				leftLegDirection = normalize(leftNeighbor->point_ - obstacle1->point_);
				isLeftLegForeign = true;
			}

			if (obstacle2->isConvex_ && det(rightLegDirection, rightNeighbor->point_ - obstacle2->point_) < 0.0f) {
				/* Leg points into obstacle. */
				rightLegDirection = normalize(rightNeighbor->point_ - obstacle2->point_);
				isRightLegForeign = true;
			}

			/* Compute cut-off centers. */
			const Vector2 leftCutoff = invTimeHorizonObst * (obstacle1->point_ - position_);
			const Vector2 rightCutoff = invTimeHorizonObst * (obstacle2->point_ - position_);
			const Vector2 cutoffVec = rightCutoff - leftCutoff;

			/* Project current velocity on velocity obstacle. */
			Line line;

			const float t = (obstacle1 == obstacle2 ? 0.5f : ((velocity_ - leftCutoff) * cutoffVec) / absSq(cutoffVec));
			const float tLeft = ((velocity_ - leftCutoff) * leftLegDirection);
			const float tRight = ((velocity_ - rightCutoff) * rightLegDirection);

			if ((t < 0.0f && tLeft < 0.0f) || (obstacle1 == obstacle2 && tLeft < 0.0f && tRight < 0.0f)) {
				/* Project on left cut-off circle. */
				const Vector2 unitW = normalize(velocity_ - leftCutoff);

				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = leftCutoff + radius_ * invTimeHorizonObst * unitW;
			}
			else if (t > 1.0f && tRight < 0.0f) {
				/* Project on right cut-off circle. */
				const Vector2 unitW = normalize(velocity_ - rightCutoff);

				line.direction = Vector2(unitW.y(), -unitW.x());
				line.point = rightCutoff + radius_ * invTimeHorizonObst * unitW;
			}
			else {
				/*
				 * Project on left leg, right leg, or cut-off line, whichever is closest
				 * to velocity.
				 */
				const float distSqCutoff = ((t < 0.0f || t > 1.0f || obstacle1 == obstacle2) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + t * cutoffVec)));
				const float distSqLeft = ((tLeft < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (leftCutoff + tLeft * leftLegDirection)));
				const float distSqRight = ((tRight < 0.0f) ? std::numeric_limits<float>::infinity() : absSq(velocity_ - (rightCutoff + tRight * rightLegDirection)));

				if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight) {
					/* Project on cut-off line. */
					line.direction = -normalize(cutoffVec);
					line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
				}
				else if (distSqLeft <= distSqRight) {
					/* Project on left leg. */
					if (isLeftLegForeign) {
						continue;
					}
					else {
						line.direction = leftLegDirection;
						line.point = leftCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
					}
				}
				else {
					/* Project on right leg. */
					if (isRightLegForeign) {
						continue;
					}
					else {
						line.direction = -rightLegDirection;
						line.point = rightCutoff + radius_ * invTimeHorizonObst * Vector2(-line.direction.y(), line.direction.x());
					}
				}
			}

			orcaLines_.push_back(line);
		}

		const size_t numObstLines = orcaLines_.size();


		const float invTimeHorizon = 1.0f / timeHorizon_;

		/* Create agent ORCA lines. */
		for (size_t i = 0; i < agentNeighbors_.size(); ++i) {
			const Agent *const other = sim_->agents_[agentNeighbors_[i].second];

			const Vector2 relativePosition = other->position_ - position_;
			const Vector2 relativeVelocity = velocity_ - other->velocity_;
			const float distSq = absSq(relativePosition);
			const float combinedRadius = radius_ + other->radius_;
			const float combinedRadiusSq = sqr(combinedRadius);

			Line line;
			Vector2 u;

			if (distSq > combinedRadiusSq) {
				/* No collision. */
				const Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;
				/* Vector from cutoff center to relative velocity. */
				const float wLengthSq = absSq(w);

				const float dotProduct1 = w * relativePosition;

				if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) {
					/* Project on cut-off circle. */
					const float wLength = std::sqrt(wLengthSq);
					const Vector2 unitW = w / wLength;

					line.direction = Vector2(unitW.y(), -unitW.x());
					u = (combinedRadius * invTimeHorizon - wLength) * unitW;
				}
				else {
					/* Project on legs. */
					const float leg = std::sqrt(distSq - combinedRadiusSq);

					if (det(relativePosition, w) > 0.0f) {
						/* Project on left leg. */
						line.direction = Vector2(relativePosition.x() * leg - relativePosition.y() * combinedRadius, relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}
					else {
						/* Project on right leg. */
						line.direction = -Vector2(relativePosition.x() * leg + relativePosition.y() * combinedRadius, -relativePosition.x() * combinedRadius + relativePosition.y() * leg) / distSq;
					}

					const float dotProduct2 = relativeVelocity * line.direction;

					u = dotProduct2 * line.direction - relativeVelocity;
				}
			}
			else {
				/* Collision. */
				const float dist = std::sqrt(distSq);
				const Vector2 unitRelativePosition = relativePosition / dist;

				line.direction = Vector2(-unitRelativePosition.y(), unitRelativePosition.x());

				const Vector2 point = ((dist - combinedRadius) / sim_->timeStep_) * unitRelativePosition;
				const float dotProduct = (relativeVelocity - point) * line.direction;

				u = point + dotProduct * line.direction - relativeVelocity;
			}

			line.point = velocity_ + 0.5f * u;

			orcaLines_.push_back(line);
		}

		if (!linearProgram2(orcaLines_, orcaLines_.size(), maxSpeed_, prefVelocity_, false, newVelocity_)) {
			linearProgram3(orcaLines_, numObstLines, maxSpeed_, newVelocity_);
		}
	}

	void Agent::insertAgentNeighbor(size_t agentNo, float &rangeSq)
	{
		const Agent *const other = sim_->agents_[agentNo];

		if (this != other) {
			const float distSq = absSq(position_ - other->position_);

			if (distSq < rangeSq) {
				if (agentNeighbors_.size() < maxNeighbors_) {
					agentNeighbors_.push_back(std::make_pair(0.0f, 0));
				}

				size_t i = agentNeighbors_.size() - 1;

				while (i != 0 && distSq < agentNeighbors_[i - 1].first) {
					agentNeighbors_[i] = agentNeighbors_[i - 1];
					--i;
				}

				agentNeighbors_[i] = std::make_pair(distSq, agentNo);

				if (agentNeighbors_.size() == maxNeighbors_) {
					rangeSq = agentNeighbors_.back().first;
				}
			}
		}
	}

	void Agent::insertObstacleNeighbor(size_t obstacleNo, float rangeSq)
	{
		const Obstacle *const obstacle = sim_->obstacles_[obstacleNo];
		const Obstacle *const nextObstacle = sim_->obstacles_[obstacle->nextObstacle_];

		const float distSq = distSqPointLineSegment(obstacle->point_, nextObstacle->point_, position_);

		if (distSq < rangeSq) {
			obstacleNeighbors_.push_back(std::make_pair(0.0f, 0));

			size_t i = obstacleNeighbors_.size() - 1;

			while (i != 0 && distSq < obstacleNeighbors_[i - 1].first) {
				obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
				--i;
			}

			obstacleNeighbors_[i] = std::make_pair(distSq, obstacleNo);
		}
	}

	void Agent::update()
	{
		velocity_ = newVelocity_;
		position_ += velocity_ * sim_->timeStep_;
	}

	void Agent::update(const Vector2 &newVelocity_, const Vector2 &newPosition_) {
		velocity_ = newVelocity_;
		position_ = newPosition_;
	}

	bool linearProgram1(const std::vector<Line> &lines, size_t lineNo, float radius, const Vector2 &optVelocity, bool directionOpt, Vector2 &result)
	{
		const float discriminant = sqr(radius) - sqr(det(lines[lineNo].direction, lines[lineNo].point));

		if (discriminant < 0.0f) {
			/* Max speed circle fully invalidates line lineNo. */
			return false;
		}

		const float sqrtDiscriminant = std::sqrt(discriminant);

		float tLeft = -(lines[lineNo].direction * lines[lineNo].point) - sqrtDiscriminant;

		float tRight = -(lines[lineNo].direction * lines[lineNo].point) + sqrtDiscriminant;

		for (size_t i = 0; i < lineNo; ++i) {
			const float determinant = det(lines[lineNo].direction, lines[i].direction);

			if (std::fabs(determinant) <= RVO_EPSILON) {
				/* Lines lineNo and i are (almost) parallel. */
				if (det(lines[i].direction, lines[lineNo].point - lines[i].point) < 0.0f) {
					/* Line i fully invalidates line lineNo. */
					return false;
				}
				else {
					/* Line i does not impose constraint on line lineNo. */
					continue;
				}
			}

			const float t = det(lines[i].direction, lines[lineNo].point - lines[i].point) / determinant;

			if (determinant > 0.0f) {
				/* Line i bounds line lineNo on the right. */
				tRight = std::min(tRight, t);
			}
			else {
				/* Line i bounds line lineNo on the left. */
				tLeft = std::max(tLeft, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity * lines[lineNo].direction > 0.0f) {
				/* Take right extreme. */
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				/* Take left extreme. */
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
		}
		else {
			/* Optimize closest point. */
			const float t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

			if (t < tLeft) {
				result = lines[lineNo].point + tLeft * lines[lineNo].direction;
			}
			else if (t > tRight) {
				result = lines[lineNo].point + tRight * lines[lineNo].direction;
			}
			else {
				result = lines[lineNo].point + t * lines[lineNo].direction;
			}
		}

		return true;
	}

	bool linearProgram2(const std::vector<Line> &lines, size_t num, float radius, const Vector2 &optVelocity, bool directionOpt, Vector2 &result)
	{
		if (directionOpt) {
			/*
			 * Optimize direction. Note that the optimization velocity is of unit
			 * length in this case.
			 */
			result = optVelocity * radius;
		}
		else if (absSq(optVelocity) > sqr(radius)) {
			/* Optimize closest point and outside circle. */
			result = normalize(optVelocity) * radius;
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
		}

		for (size_t i = 0; i < num; ++i) {
			if (det(lines[i].direction, result - lines[i].point) < 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result)) {
					return false;
				}
			}
		}

		return true;
	}

	void linearProgram3(const std::vector<Line> &lines, size_t numObstLines, float radius, Vector2 &result)
	{
		if (!linearProgram2(lines, numObstLines, radius, Vector2(), false, result)) {
			/*
			 * This should not happen unless agents are initialized within obstacle
			 * and time step is small.
			 */
			linearProgram2(lines, numObstLines, std::numeric_limits<float>::infinity(), Vector2(), false, result);

			return;
		}

		float distance = 0.0f;

		for (size_t i = numObstLines; i < lines.size(); ++i) {
			if (det(lines[i].direction, lines[i].point - result) > distance) {
				/* Result does not satisfy constraint of line i. */
				std::vector<Line> projLines(lines.begin(), lines.begin() + static_cast<ptrdiff_t>(numObstLines));

				for (size_t j = numObstLines; j < i; ++j) {
					Line line;

					float determinant = det(lines[i].direction, lines[j].direction);

					if (std::fabs(determinant) <= RVO_EPSILON) {
						/* Line i and line j are (almost) parallel. */
						if (lines[i].direction * lines[j].direction > 0.0f) {
							/* Line i and line j point in the same direction. */
							continue;
						}
						else {
							/* Line i and line j point in opposite direction. */
							line.point = 0.5f * (lines[i].point + lines[j].point);
						}
					}
					else {
						line.point = lines[i].point + (det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
					}

					line.direction = normalize(lines[j].direction - lines[i].direction);
					projLines.push_back(line);
				}

				const Vector2 tempResult = result;

				if (!linearProgram2(projLines, projLines.size(), radius, Vector2(-lines[i].direction.y(), lines[i].direction.x()), true, result)) {
					/* This should in principle not happen.  The result is by definition
					 * already in the feasible region of this linear program. If it fails,
					 * it is due to small floating point error, and the current result is
					 * kept.
					 */
					result = tempResult;
				}

				distance = det(lines[i].direction, lines[i].point - result);
			}
		}
	}
}
