--- A simple A star implementation based on the hybrid A star. The difference is that the state space isn't really
--- 3 dimensional as we do not take the heading into account and we use a different set of motion primitives which
--- puts us on the grid points.
---@class AStar : HybridAStar
AStar = CpObject(PathfinderInterface)

function AStar:init(vehicle, yieldAfter, maxIterations)
    self.vehicle = vehicle
    self.count = 0
    self.yields = 0
    self.yieldAfter = yieldAfter or 200
    self.maxIterations = maxIterations or 20000
    self.path = {}
    self.iterations = 0
    -- this needs to be small enough that no vehicle fit between the grid points (and remain undetected)
    self.deltaPos = 3
    self.deltaPosGoal = self.deltaPos
    self.deltaThetaGoal = math.pi
    --- A simple set of motion primitives to use with an A* algorithm, pointing to 8 directions, to the 8 grid neighbors
    self.primitives = {}
    local d = self.deltaPos
    local dSqrt2 = math.sqrt(2) * d
    table.insert(self.primitives, { dx = d, dy = 0, dt = 0, d = d, })
    table.insert(self.primitives, { dx = d, dy = d, dt = 1 * math.pi / 4, d = dSqrt2, })
    table.insert(self.primitives, { dx = 0, dy = d, dt = 2 * math.pi / 4, d = d, })
    table.insert(self.primitives, { dx = -d, dy = d, dt = 3 * math.pi / 4, d = dSqrt2, })
    table.insert(self.primitives, { dx = -d, dy = 0, dt = 4 * math.pi / 4, d = d, })
    table.insert(self.primitives, { dx = -d, dy = -d, dt = 5 * math.pi / 4, d = dSqrt2, })
    table.insert(self.primitives, { dx = 0, dy = -d, dt = 6 * math.pi / 4, d = d, })
    table.insert(self.primitives, { dx = d, dy = -d, dt = 7 * math.pi / 4, d = dSqrt2, })
end

--- A* successors are simply the grid neighbors
function AStar:createSuccessor(node, primitive)
    local xSucc = node.x + primitive.dx
    local ySucc = node.y + primitive.dy
    local tSucc = primitive.dt
    return State3D(xSucc, ySucc, tSucc, node.g, node, Gear.Forward, Steer.Straight, 0, node.d + primitive.d)
end

--- Starts a pathfinder run. This initializes the pathfinder and then calls resume() which does the real work.
---@param start State3D start node
---@param goal State3D goal node
---@param constraints PathfinderConstraintInterface constraints (validity, penalty) for the pathfinder
--- must have the following functions defined:
---   getNodePenalty() function get penalty for a node, see getNodePenalty()
---   isValidNode()) function function to check if a node should even be considered
---   isValidAnalyticSolutionNode()) function function to check if a node of an analytic solution should even be considered.
---                              when we search for a valid analytic solution we use this instead of isValidNode()
---@return boolean, [State3D]|nil, boolean done, path, goal node invalid
function AStar:initRun(start, goal, turnRadius, allowReverse, constraints)
    self:debug('Start A* pathfinding between %s and %s', tostring(start), tostring(goal))
    self.goal = goal
    self.constraints = constraints

    -- create the open list for the nodes as a binary heap where
    -- the node with the lowest total cost is at the top
    self.openList = BinaryHeap.minUnique(function(a, b)
        return a:lt(b)
    end)

    -- create the configuration space
    ---@type HybridAStar.NodeList closedList
    self.nodes = HybridAStar.NodeList(self.deltaPos, 360)

    -- ignore trailer for the first check, we don't know its heading anyway
    if not constraints:isValidNode(goal, true) then
        self:debug('Goal node is invalid, abort pathfinding.')
        return true, nil, true
    end

    start:insert(self.openList)

    self.iterations = 0
    self.yields = 0
    self.initialized = true
    return false
end

--- Reentry-safe pathfinder runner
---@return boolean true if the pathfinding is done, false if it isn't ready. In this case you'll have to call resume() again
---@return table|nil the path if found as array of State3D
---@return boolean goal node invalid
function AStar:run(start, goal, turnRadius, allowReverse, constraints)
    if not self.initialized then
        local done, path, goalNodeInvalid = self:initRun(start, goal, turnRadius, allowReverse, constraints)
        if done then
            return done, path, goalNodeInvalid
        end
    end
    self.timer = openIntervalTimer()
    while self.openList:size() > 0 and self.iterations < self.maxIterations do
        -- pop lowest cost node from queue
        ---@type State3D
        local pred = State3D.pop(self.openList)
        --self:debug('pop %s', tostring(pred))

        if pred:equals(self.goal, self.deltaPosGoal, self.deltaThetaGoal) then
            -- done!
            self:debug('Popped the goal (%d).', self.iterations)
            return self:finishRun(true, self:rollUpPath(pred, self.goal))
        end
        self.count = self.count + 1
        -- yield after the configured iterations or after 20 ms
        if (self.count % self.yieldAfter == 0 or readIntervalTimerMs(self.timer) > 20) then
            self.yields = self.yields + 1
            closeIntervalTimer(self.timer)
            -- if we had the coroutine package, we would coursePlayCoroutine.yield(false) here
            return false
        end
        if not pred:isClosed() then
            -- create the successor nodes
            for _, primitive in ipairs(self.primitives) do
                ---@type State3D
                local succ = self:createSuccessor(pred, primitive, self.hitchLength)
                if succ:equals(self.goal, self.deltaPosGoal, self.deltaThetaGoal) then
                    succ.pred = succ.pred
                    self:debug('Successor at the goal (%d).', self.iterations)
                    return self:finishRun(true, self:rollUpPath(succ, self.goal))
                end
                self:expand(succ, primitive)
            end
            -- node as been expanded, close it to prevent expansion again
            --self:debug(tostring(pred))
            pred:close()
        end
        self.iterations = self.iterations + 1
        if self.iterations % 1000 == 0 then
            self:debug('iteration %d...', self.iterations)
            self.constraints:showStatistics()
        end
        local r = self.iterations / self.maxIterations
    end
    --self:printOpenList(self.openList)
    self:debug('No path found: iterations %d, yields %d, cost %.1f ', self.iterations, self.yields, self.nodes.lowestCost)
    return self:finishRun(true, nil)
end

function AStar:expand(succ, primitive)
    local existingSuccNode = self.nodes:get(succ)
    if not existingSuccNode or (existingSuccNode and not existingSuccNode:isClosed()) then
        if self.constraints:isValidNode(succ) then
            succ:updateG(primitive, self.constraints:getNodePenalty(succ))
            -- 1.5 times distance to goal heuristic to make it faster (but less accurately follow the shortest path)
            succ:updateH(self.goal, 0, succ:distance(self.goal) * 1.5)
            if existingSuccNode then
                -- there is already a node at this (discretized) position
                -- add a small number before comparing to adjust for floating point calculation differences
                if existingSuccNode:getCost() + 0.001 >= succ:getCost() then
                    -- the path to the existing node is more expensive than the new one, so replace it
                    -- with the new one
                    if self.openList:valueByPayload(existingSuccNode) then
                        -- existing node is on open list already, remove it from there
                        existingSuccNode:remove(self.openList)
                    end
                    -- add (update) to the state space
                    self.nodes:put(succ)
                    -- add the new, cheaper node to the open list
                    succ:insert(self.openList)
                else
                    --self:debug('insert existing node back %s (iteration %d), diff %s', tostring(succ), self.iterations, tostring(succ:getCost() - existingSuccNode:getCost()))
                end
            else
                -- successor cell does not yet exist
                self.nodes:put(succ)
                -- put it on the open list as well
                succ:insert(self.openList)
            end
        else
            -- invalid node, close to prevent expansion
            succ:close()
        end -- valid node
    end
end

--- Wrap up this run, clean up timer, reset initialized flag so next run will start cleanly
function AStar:finishRun(result, path)
    self.initialized = false
    self.constraints:showStatistics()
    closeIntervalTimer(self.timer)
    return result, path
end

---@param node State3D
function AStar:rollUpPath(node, goal, path)
    path = path or {}
    local currentNode = node
    self:debug('Goal node at %.2f/%.2f, cost %.1f (%.1f - %.1f)', goal.x, goal.y, node.cost,
            self.nodes.lowestCost, self.nodes.highestCost)
    table.insert(path, 1, currentNode)
    local nSkippedNodes = 0
    self.nPenaltyCalls = 0
    -- cache for penalties to make post smoothing faster, by calling getNodePenalty() only once per grid cell
    self.penalties = HybridAStar.NodeList(self.deltaPos, 360)
    while currentNode.pred do
        -- smoothing the path
        if currentNode.pred.pred then
            -- if the predecessor has a predecessor, then check if we can skip the predecessor and go directly
            -- to its parent. This will eliminate the zig-zag in the path when moving in a direction not aligned
            -- with the grid (or its diagonal)
            if self:isObstacleBetween(path[1], currentNode.pred.pred) then
                table.insert(path, 1, currentNode.pred)
            else
                nSkippedNodes = nSkippedNodes + 1
            end
        else
            table.insert(path, 1, currentNode.pred)
        end
        currentNode = currentNode.pred
    end
    self:debug('Nodes %d (skipped %d for smoothing), iterations %d, yields %d, penalty calls %d',
            #path, nSkippedNodes, self.iterations, self.yields, self.nPenaltyCalls)
    -- now that we straightened the path, we may end up with just 2 nodes, start and end, so let's add
    -- some in between
    return self:addIntermediatePoints(path)
end

---@param func function(x, y, ...) function to call for each point. If it returns anything
--- other than nil, the loop is stopped and the result is returned.
function AStar:runForImmediatePoints(n1, n2, func, ...)
    local x1, y1, x2, y2 = n1.x, n1.y, n2.x, n2.y
    local dx, dy = x2 - x1, y2 - y1
    local d = math.sqrt(dx * dx + dy * dy)
    local steps = math.floor(d / self.deltaPos)
    local stepX, stepY = dx / steps, dy / steps
    for i = 1, steps do
        local x, y = x1 + i * stepX, y1 + i * stepY
        local result = func(x, y, ...)
        if result ~= nil then
            return result
        end
    end
end

--- Very conservatively check, if we can make a shortcut between two nodes, to smooth (straighten) the raw A* path.
--- If there is any node penalty, we consider it an obstacle and thus won't skip this node
---@param n1 State3D
---@param n2 State3D
function AStar:isObstacleBetween(n1, n2)
    return self:runForImmediatePoints(n1, n2,
            function(x, y)
                local node = State3D(x, y, 0)
                local penalty
                local cachedPenalty = self.penalties:get(node)
                if cachedPenalty then
                    penalty = cachedPenalty.penalty
                else
                    self.nPenaltyCalls = self.nPenaltyCalls + 1
                    penalty = self.constraints:getNodePenalty(node)
                    node.penalty = penalty
                    self.penalties:put(node)
                end
                if penalty > 0 then
                    return true
                end
            end)
end

function AStar:addIntermediatePoints(path)
    local newPath = {}
    for i = 1, #path - 1 do
        local n1, n2 = path[i], path[i + 1]
        table.insert(newPath, n1)
        self:runForImmediatePoints(n1, n2,
                function(x, y)
                    table.insert(newPath, State3D(x, y, 0))
                end)
    end
    table.insert(newPath, path[#path])
    return newPath
end

function AStar:nodeIterator()
    return self.nodes and self.nodes:iterator() or function()  end
end