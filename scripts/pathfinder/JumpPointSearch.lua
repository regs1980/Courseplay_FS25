---@class HybridAStar.JpsMotionPrimitives : HybridAStar.SimpleMotionPrimitives
HybridAStar.JpsMotionPrimitives = CpObject(HybridAStar.SimpleMotionPrimitives)
function HybridAStar.JpsMotionPrimitives:init(gridSize, deltaPosGoal, deltaThetaGoal, constraints, goal)
    -- similar to the A*, the possible motion primitives (the neighbors) are in all 8 directions.
    self.primitives = {}

    self.gridSize = gridSize
    local d = gridSize
    table.insert(self.primitives, Vector(d, 0))
    table.insert(self.primitives, Vector(d, d))
    table.insert(self.primitives, Vector(0, d))
    table.insert(self.primitives, Vector(-d, d))

    table.insert(self.primitives, Vector(-d, 0))
    table.insert(self.primitives, Vector(-d, -d))
    table.insert(self.primitives, Vector(0, -d))
    table.insert(self.primitives, Vector(d, -d))

    -- set up the pruning rules for each primitive, that is, if we used this primitive to move to the
    -- next node, which neighbors of that node should be examined further.
    -- right
    self.primitives[1].nextPrimitives = {
        -- unconditionally include natural neighbors.
        -- always need this neighbor to the left or right of the current node
        { check = nil, nextPrimitive = self.primitives[1] },
        -- if there is an obstacle at 'check' if we got to the node through this primitive, then
        -- create a forced neighbors at 'forcedNeighbor'
        { check = Vector(0, self.gridSize), nextPrimitive = self.primitives[2] },
        { check = Vector(0, -self.gridSize), nextPrimitive = self.primitives[8] }
    }
    -- left
    self.primitives[5].nextPrimitives = {
        -- unconditionally include natural neighbors.
        -- always need this neighbor to the left or right of the current node
        { check = nil, nextPrimitive = self.primitives[5] },
        -- if there is an obstacle at 'check' if we got to the node through this primitive, then
        -- create a forced neighbors at 'forcedNeighbor'
        { check = Vector(0, self.gridSize), nextPrimitive = self.primitives[4] },
        { check = Vector(0, -self.gridSize), nextPrimitive = self.primitives[6] }
    }

    -- up
    self.primitives[3].nextPrimitives = {
        -- unconditionally include natural neighbors.
        -- always need this neighbor to the left or right of the current node
        { check = nil, nextPrimitive = self.primitives[3] },
        -- forced neighbors are included only if the checked neighbors are invalid
        { check = Vector(self.gridSize, 0), nextPrimitive = self.primitives[2] },
        { check = Vector(-self.gridSize, 0), nextPrimitive = self.primitives[4] }
    }

    -- down
    self.primitives[7].nextPrimitives = {
        -- unconditionally include natural neighbors.
        -- always need this neighbor to the left or right of the current node
        { check = nil, nextPrimitive = self.primitives[7] },
        -- forced neighbors are included only if the checked neighbors are invalid
        { check = Vector(self.gridSize, 0), nextPrimitive = self.primitives[8] },
        { check = Vector(-self.gridSize, 0), nextPrimitive = self.primitives[6] }
    }

    -- up right
    self.primitives[2].nextPrimitives = {
        -- unconditionally include natural neighbors.
        -- we always need this neighbor to the left or right of the current node, plus the one in the same direction
        { check = nil, nextPrimitive = self.primitives[2] },
        { check = nil, nextPrimitive = self.primitives[1] },
        { check = nil, nextPrimitive = self.primitives[3] },
        -- forced neighbors are included only if the checked neighbors are invalid
        { check = Vector(-self.gridSize, 0), nextPrimitive = self.primitives[4] },
        { check = Vector(0, -self.gridSize), nextPrimitive = self.primitives[8] },
    }

    -- up left
    self.primitives[4].nextPrimitives = {
        -- unconditionally include natural neighbors.
        -- we always need this neighbor to the left or right of the current node, plus the one in the same direction
        { check = nil, nextPrimitive = self.primitives[4] },
        { check = nil, nextPrimitive = self.primitives[3] },
        { check = nil, nextPrimitive = self.primitives[5] },
        -- forced neighbors are included only if the checked neighbors are invalid
        { check = Vector(self.gridSize, 0), nextPrimitive = self.primitives[2] },
        { check = Vector(0, -self.gridSize), nextPrimitive = self.primitives[6] },
    }

    -- down left
    self.primitives[6].nextPrimitives = {
        -- unconditionally include natural neighbors.
        -- we always need this neighbor to the left or right of the current node, plus the one in the same direction
        { check = nil, nextPrimitive = self.primitives[6] },
        { check = nil, nextPrimitive = self.primitives[5] },
        { check = nil, nextPrimitive = self.primitives[7] },
        -- forced neighbors are included only if the checked neighbors are invalid
        { check = Vector(self.gridSize, 0), nextPrimitive = self.primitives[8] },
        { check = Vector(0, self.gridSize), nextPrimitive = self.primitives[4] },
    }

    -- down right
    self.primitives[8].nextPrimitives = {
        -- unconditionally include natural neighbors.
        -- we always need this neighbor to the left or right of the current node, plus the one in the same direction
        { check = nil, nextPrimitive = self.primitives[8] },
        { check = nil, nextPrimitive = self.primitives[1] },
        { check = nil, nextPrimitive = self.primitives[7] },
        -- forced neighbors are included only if the checked neighbors are invalid
        { check = Vector(-self.gridSize, 0), nextPrimitive = self.primitives[6] },
        { check = Vector(0, self.gridSize), nextPrimitive = self.primitives[2] },
    }

    for i = 1, #self.primitives do
        -- if we moved to the current node using primitive[i], then the primitive we would need to go back to
        -- the parent is primitive[i].parent
        self.primitives[i].parent = self.primitives[i + 4] or self.primitives[i - 4]
        -- cache the length
        self.primitives[i].d = self.primitives[i]:length()
    end
    self.deltaPosGoal = deltaPosGoal
    self.deltaThetaGoal = deltaThetaGoal
    self.constraints = constraints
    self.goal = goal
end

function HybridAStar.JpsMotionPrimitives:getGridSize()
    return self.gridSize
end

function HybridAStar.JpsMotionPrimitives:isValidNode(x, y, t, constraints)
    local node = { x = x, y = y, t = t }
    return constraints:isValidNode(node, true, true)
end

---@param node State3D
---@param primitive
function HybridAStar.JpsMotionPrimitives:getNextPrimitives(node, primitive)
    local nextPrimitives, forced = {}, false
    for _, n in ipairs(primitive.nextPrimitives) do
        if n.check then
            -- check if the neighbor is valid
            local neighborToCheck = State3D(node.x, node.y, n.check:heading()) + n.check
            if not self.constraints:isValidNode(neighborToCheck, true, true) then
                -- we have a forced neighbor
                table.insert(nextPrimitives, n.nextPrimitive)
                forced = true
            end
        else
            table.insert(nextPrimitives, n.nextPrimitive)
        end
    end
    return nextPrimitives, forced
end

-- Get the possible neighbors when coming from the predecessor node.
-- While the other HybridAStar derived algorithms use a real motion primitive, meaning it gives the relative
-- x, y and theta values which need to be added to the predecessor, JPS supplies the actual coordinates of
-- the successors here instead.
-- This is not the most elegant solution and the only reason we do this is to be able to reuse the whole
-- framework in HybridAStar.lua with JPS.
function HybridAStar.JpsMotionPrimitives:getPrimitives(node)
    local primitives
    if node.primitive then
        -- the primitive that was used to get to this node
        primitives = self:getNextPrimitives(node, node.primitive)
    else
        -- first node, no predecessor, no incoming primitive
        primitives = self.primitives
    end
    local jumpNodes = {}
    for _, p in ipairs(primitives) do
        local jumpNode = self:jump(node, p)
        if jumpNode then
            -- an ugly hack here, HybridAStar uses primitive.d to calculate the g cost but we pass the
            -- actual successor node, not the primitive, so
            table.insert(jumpNodes, State3D(jumpNode.x, jumpNode.y, jumpNode.t, node.g, node, Gear.Forward, Steer.Straight,
                    nil, (jumpNode - node):length(), p))
        end
    end
    return jumpNodes
end

function HybridAStar.JpsMotionPrimitives:jump(node, primitive, recursionCounter)
    recursionCounter = recursionCounter and recursionCounter + 1 or 1
    if recursionCounter > 5 then
        return node
    end
    local v = node + primitive
    local successor = State3D(v.x, v.y, primitive:heading())
    if not self.constraints:isValidNode(successor) then
        return nil
    end
    if successor:equals(self.goal, self.deltaPosGoal, self.deltaThetaGoal) then
        return successor
    end

    local _, forced = self:getNextPrimitives(successor, primitive)
    if forced then
        return successor
    end

    if primitive.x ~= 0 and primitive.y ~= 0 then
        -- diagonal move, must check first horizontally and vertically
        -- left or right
        local jumpPoint = self:jump(successor, primitive.x > 0 and self.primitives[1] or self.primitives[5])
        if jumpPoint then
            return successor
        else
            -- up or down
            jumpPoint = self:jump(successor, primitive.y > 0 and self.primitives[3] or self.primitives[7])
            if jumpPoint then
                return successor
            end
        end
    end

    return self:jump(successor, primitive, recursionCounter)
end

function HybridAStar.JpsMotionPrimitives:createSuccessor(node, jumpNode, hitchLength)
    jumpNode.tTrailer = node:getNextTrailerHeading(self.gridSize, hitchLength)
    return jumpNode
end
--- A Jump Point Search
---@class JumpPointSearch : AStar
JumpPointSearch = CpObject(AStar)
JumpPointSearch.markers = {}

function JumpPointSearch:init(vehicle, yieldAfter, maxIterations)
    AStar.init(self, vehicle, yieldAfter, maxIterations)
end

function JumpPointSearch:initRun(start, goal, turnRadius, allowReverse, constraints, hitchLength)
    self.logger:setLevel(Logger.level.trace)
    return AStar.initRun(self, start, goal, turnRadius, allowReverse, constraints, hitchLength)
end

function JumpPointSearch:getMotionPrimitives(turnRadius, allowReverse)
    return HybridAStar.JpsMotionPrimitives(self.deltaPos, self.deltaPosGoal, self.deltaThetaGoal, self.constraints, self.goal)
end

--[[function JumpPointSearch:rollUpPath(node, goal, path)
    return HybridAStar.rollUpPath(self, node, goal, path)
end]]

---@class HybridAStarWithJpsInTheMiddle : HybridAStarWithAStarInTheMiddle
HybridAStarWithJpsInTheMiddle = CpObject(HybridAStarWithAStarInTheMiddle)

function HybridAStarWithJpsInTheMiddle:init(hybridRange, yieldAfter, maxIterations, mustBeAccurate)
    HybridAStarWithAStarInTheMiddle.init(self, hybridRange, yieldAfter, maxIterations, mustBeAccurate)
end

function HybridAStarWithJpsInTheMiddle:getAStar()
    return JumpPointSearch(self.yieldAfter)
end

