using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

namespace _9.Генерация_биомов___Графы_и_проектирование
{
    class Program
    {
        static void Main(string[] args)
        {
            //генерация стандартного биома
            Biom MyBiom = new Biom();
            MyBiom.Show();
            MyBiom.BuildRoad();

            //генерация биома с конфигурацией
            //-задан размер: 30x60
            //-задано условие: лава c землёй не размещаются ближе, чем в 10 клетках
            //-задана заливка тайлам земли: лужами по углам карты
            //-из генерации исключены тайлы воды
            //-задана своя пользовательская функция размещения для моста
            Biom MyBiomConfigured = new Biom.BiomConfigurator()
                .SetSize(30,60)
                .SetTilesAdjacency<Lava, Ground>(10)
                .SetFillFuncForTile<Ground>(MapFiller.FillPuddlesAtCorner)                
                .RemoveTile<Water>()
                .SetFillFuncForTile<Bridge>((map, tile, matrix) =>
                {                    
                    for (int i = 0, j = 0; i < map.Length && j < map[i].Length; i++, j++)
                    {
                        tile.PlaceInstance(map, i, 0, matrix);
                        tile.PlaceInstance(map, map.Length - 1, j, matrix);
                        tile.PlaceInstance(map, map.Length - 1, map[0].Length - 1 - j, matrix);
                    }
                })
                .Create();            

            //MyBiomConfigured.Show();            
            //MyBiomConfigured.BuildRoad();            

            Console.Read();
        }
    }

    public class Biom
    {
        private Tile[][] _map;
        private AdjacencyMatrix _matrix = new AdjacencyMatrix();

        public int Height { get; private set; }
        public int Width { get; private set; }

        private readonly Dictionary<Type, char> TilesConsoleViewDic = new Dictionary<Type, char>()
        {
            { typeof(Ground), '.' },
            { typeof(Water), '0' },
            { typeof(Lava), '+' },
            { typeof(Bridge), '█' },
            { typeof(Road), '░' },
        };

        private Dictionary<Type, Action<Tile[][], Tile, AdjacencyMatrix>> _fillFuncsForTiles = new Dictionary<Type, Action<Tile[][], Tile, AdjacencyMatrix>>()
        {
            { typeof(Ground), MapFiller.FillAll },
            { typeof(Water), MapFiller.FillPuddles },
            { typeof(Lava), MapFiller.FillPuddles },
            { typeof(Bridge), MapFiller.FillRandom },
            { typeof(Road), null },
        };

        private Dictionary<Type, Tile> _tileSet = new Dictionary<Type, Tile>()
        {
            { typeof(Ground), new Ground() },
            { typeof(Water), new Water() },
            { typeof(Lava), new Lava() },
            { typeof(Bridge), new Bridge() },
        };        

        public Biom(int height = 100, int width = 100)
        {
            if (height < 5 || width < 5)
                throw new ArgumentException("Height and width must be greater then 2");            

            Height = height;
            Width = width;
            
            GenerateBiom();            
        }

        public class BiomConfigurator
        {
            private Biom _biom;
            
            public BiomConfigurator()
            {
                _biom = new Biom();
            }                        

            /// <summary>
            /// Задать размер, не меньше 5x5.
            /// </summary>
            public BiomConfigurator SetSize(int height, int width)
            {
                if (height < 5 || width < 5)
                    throw new ArgumentException();
                
                _biom.Width = width;
                _biom.Height = height;

                return this;
            }

            /// <summary>
            /// Задать допустимую дистанцию для размещения двух типов тайлов.            
            /// allowedDistance должна быть больше 0, типы тайлов должны реализовывать Tile.
            /// </summary>            
            public BiomConfigurator SetTilesAdjacency<T1, T2>(int allowedDistance)
                where T1: Tile
                where T2: Tile
            {
                if (allowedDistance < 0)
                    throw new ArgumentException();

                _biom._matrix.SetTilesAdjacency<T1,T2>(allowedDistance);

                return this;
            }

            /// <summary>
            /// Задать функцию генерации для типа тайла.            
            /// Тип тайла должен реализовывать Tile, функцию заливки можно выбрать в MapFiller или задать свою.
            /// </summary>            
            public BiomConfigurator SetFillFuncForTile<T>(Action<Tile[][], Tile, AdjacencyMatrix> func)
                where T: Tile
            {
                if (func == null || !_biom._fillFuncsForTiles.ContainsKey(typeof(T)))
                    throw new ArgumentException();

                _biom._fillFuncsForTiles[typeof(T)] = func;

                return this;
            }

            /// <summary>
            /// Добавить тип тайлов в генерацию биома.
            /// Тип тайла должен реализовывать Tile.
            /// </summary>            
            public BiomConfigurator AddTile<T>()
                where T: Tile, new()
            {
                if (!_biom._tileSet.ContainsKey(typeof(T)))
                        _biom._tileSet.Add(typeof(T), new T());

                return this;
            }

            /// <summary>
            /// Убрать тип тайлов из генерации биома.            
            /// Тип тайла должен реализовывать Tile.
            /// </summary>            
            public BiomConfigurator RemoveTile<T>()
                where T : Tile, new()
            {
                if (_biom._tileSet.ContainsKey(typeof(T)))
                    _biom._tileSet.Remove(typeof(T));

                return this;
            }

            /// <summary>
            /// Сгенерировать биом.
            /// </summary>            
            public Biom Create()
            {
                _biom.GenerateBiom();
                return _biom;
            }            
        }

        public void Show()
        {
            foreach (var row in _map)
            {
                foreach (var tile in row)                
                    Console.Write(tile != null ? TilesConsoleViewDic[tile.GetType()] : ' ');                

                Console.WriteLine("");
            }            
        }

        public void BuildRoad()
        {            
            Console.WriteLine("\nСтрою путь! Подождите!");

            List<Tile> list = APathFinder.FindPath(_map, _map[0][0], _map[Height - 1][Width - 1]);

            Console.WriteLine("\nГОТОВО!!!");

            if (list == null)
            {
                Console.WriteLine("Не нашел пути :'(");
                return;
            }

            Console.ForegroundColor = ConsoleColor.Green;
            Road road = new Road();
            foreach (var tile in list)
            {
                Console.SetCursorPosition(tile.Y, tile.X);

                if (tile.GetType() != typeof(Bridge))
                {
                    road.PlaceInstance(_map, tile.X, tile.Y, _matrix);
                    Console.Write('░');
                }
                else                
                    Console.Write('█');

                Thread.Sleep(10);
            }
        }

        private void GenerateBiom()
        {
            //настройка карты
            _map = new Tile[Height][];
            for (int i = 0; i < Height; i++)
                _map[i] = new Tile[Width];

            //Заливка тайлами (здесь не важен порядок заполнения)
            foreach (var tile in _tileSet.Values)
            {
                var func = _fillFuncsForTiles[tile.GetType()];
                func(_map, tile, _matrix);
            }
        }
    }

    //матрица смежности
    public class AdjacencyMatrix
    {        
        //число показывает, на каком расстоянии друг от друга позволяется располагаться тайлам типов, заданных в столбцах и строках
        //0 - значит позволено накладываться друг на друга
        private int[,] _matrix =
        {
            //G  W  L  B  R
            { 0, 0, 0, 0, 0 }, //G
            { 0, 0, 5, 0, 0 }, //W
            { 0, 5, 0, 0, 0 }, //L
            { 0, 0, 0, 0, 0 }, //B
            { 0, 0, 0, 0, 0 }, //R
        };

        public int GetMaxMatrixDistanceForTile(Type type) //максимальная дистанция у заданного типа в матрице смежности на данный момент
        {
            if (!IndexByType.ContainsKey(type))
                return -1; //-1 = не найден тип в матрице

            int maxDistance = 0;
            int column = IndexByType[type];

            for (int row = 0; row < _matrix.GetLength(0); row++)
            {
                int distance = _matrix[row, column];

                if (distance > maxDistance)
                    maxDistance = distance;
            }            

            return maxDistance;
        }

        public int AllowedDistance(Type typeA, Type typeB)             
        {
            if (typeB == null) return 0;

            if (!IndexByType.ContainsKey(typeA) || !IndexByType.ContainsKey(typeB))
                return -1; //-1 = нельзя располагать

            int indexA = IndexByType[typeA];
            int indexB = IndexByType[typeB];

            return _matrix[indexA, indexB];
        }

        private Dictionary<Type, int> IndexByType = new Dictionary<Type, int>()
        {
            { typeof(Ground), 0 },
            { typeof(Water), 1 },
            { typeof(Lava), 2 },
            { typeof(Bridge), 3 },
            { typeof(Road), 4 },
        };

        public void SetTilesAdjacency<T1, T2>(int allowedDistance)
            where T1: Tile
            where T2: Tile
        {
            if (allowedDistance < 0) return;

            int index1 = IndexByType[typeof(T1)];
            int index2 = IndexByType[typeof(T2)];            

            _matrix[index1, index2] = allowedDistance;
            _matrix[index2, index1] = allowedDistance;            
        }
    }

    //разные способы размещения клеток
    public static class MapFiller
    {
        private static Random _random = new Random();

        //покрыть всю карту
        public static void FillAll(Tile[][] map, Tile tile, AdjacencyMatrix matrix)
        {
            for (int i = 0; i < map.Length; i++)
            {
                for (int j = 0; j < map[i].Length; j++)
                    tile.PlaceInstance(map, i, j, matrix);                    
            }
        }

        //покрыть случайными лужами
        public static void FillPuddles(Tile[][] map, Tile tile, AdjacencyMatrix matrix)
        {
            int maxCount = map.Length / 5;
            int count = _random.Next(5 < maxCount ? 5 : 0, map.Length / 5); //сколько всего луж

            for (int i = 0; i < count; i++)
            {
                int cx, cy;
                int tryes = 50; //количество попыток бросить лужу в рандомное место, чтоб в бесконечный луп не уходил на маленьких картах

                do
                {
                    //в координатах исключаю края карты(верхний и нижний), чтоб по ним можно было ходить (когда лужа в углу оказывается, из неё никак не выйти и не войти) //можно было бы отдельно генерить мосты для прохода в непроходимые углы, но чет уже лень
                    cx = _random.Next(1, map.Length-1);
                    cy = _random.Next(0, map[cx].Length);
                    tile.PlaceInstance(map, cx, cy, matrix);
                    tryes--;
                }
                while (map[cx][cy]?.GetType() != tile.GetType() && tryes > 0);

                int r = _random.Next(8, 20); //радиус лужи
                FillEllipse(map, tile, matrix, cx, cy, r);
            }
        }

        //покрыть случайным разбросом
        public static void FillRandom(Tile[][] map, Tile tile, AdjacencyMatrix matrix)
        {            
            int count = _random.Next(map.Length / 2, map.Length);

            for (int i = 0; i < count; i++)
            {
                int x = _random.Next(0, map.Length);
                int y = _random.Next(0, map[x].Length);
                tile.PlaceInstance(map, x, y, matrix);
            }            
        }

        //покрыть случайным плотным разбросом
        public static void FillRandomThick(Tile[][] map, Tile tile, AdjacencyMatrix matrix)
        {
            int count = map.Length * 10;

            for (int i = 0; i < count; i++)
            {
                int x = _random.Next(0, map.Length);
                int y = _random.Next(0, map[x].Length);
                tile.PlaceInstance(map, x, y, matrix);
            }
        }

        //покрыть лужами в углах карты
        public static void FillPuddlesAtCorner(Tile[][] map, Tile tile, AdjacencyMatrix matrix)
        {
            int count = 2; //сколько всего луж

            for (int i = 0; i < count; i++)
            {
                int cx = i == 0 ? 0 : map.Length-1;
                int cy = i == 0 ? 0 : map[cx].Length -1;
                
                tile.PlaceInstance(map, cx, cy, matrix);
                if (map[cx][cy]?.GetType() != tile.GetType()) continue;

                int r = _random.Next(8, 20); //радиус лужи
                FillEllipse(map, tile, matrix, cx, cy, r);
            }
        }

        //заливка эллипса заданного радиуса
        private static void FillEllipse(Tile[][] map, Tile tile, AdjacencyMatrix matrix, int cx, int cy, int radius)
        {
            while (radius > 0) //заливка от радиуса до центра
            {
                for (double t = 0; t < 2 * Math.PI; t += 0.1)
                {
                    //пропуски в заливке: чем ближе к краям - тем разреженней лужа.
                    double randomDouble = _random.NextDouble() * t;
                    if (randomDouble > 1.5 || randomDouble < 0.3) continue;

                    //заливка
                    int x = Convert.ToInt32(radius * Math.Sin(t) + cx);
                    int y = Convert.ToInt32(radius * Math.Cos(t) + cy);

                    //и здесь исключаю края карты
                    if (x >= 1 && x < map.Length - 1 && y >= 0 && y < map[x].Length && map[x][y]?.GetType() != tile.GetType())
                        tile.PlaceInstance(map, x, y, matrix);
                }
                radius--;
            }
        }
    }

    public abstract class Tile
    {
        public readonly int X;
        public readonly int Y;

        public Tile(int x, int y)
        {            
            X = x;
            Y = y;
        }

        public abstract bool IsPassable();
        public abstract void PlaceInstance(Tile[][] map, int x, int y, AdjacencyMatrix matrix);

        protected bool CheckForAllowToPlace(Tile[][] map, int x, int y, AdjacencyMatrix matrix, Type type)
        {
            if (map == null || x < 0 || x >= map.Length || y < 0 || y >= map[x].Length || matrix == null || !type.IsSubclassOf(typeof(Tile)))
                throw new ArgumentException();

            int maxDistance = matrix.GetMaxMatrixDistanceForTile(type);            

            if (maxDistance == 0)
                return true;
            else if (maxDistance == -1)
                return false;

            //проверка на максимальную дистанцию во все стороны
            int xFrom = x - maxDistance >= 0 ? x - maxDistance : 0;
            int xTo = x + maxDistance < map.Length ? x + maxDistance : map.Length;
            int yFrom = y - maxDistance >= 0 ? y - maxDistance : 0;

            for (int i = xFrom; i < xTo; i++)
            {
                int yTo = y + maxDistance < map[i].Length ? y + maxDistance : map[i].Length;
                for (int j = yFrom; j < yTo; j++)
                {
                    int allowedDistance = matrix.AllowedDistance(type, map[i][j]?.GetType());                    
                    if (allowedDistance > 0 && (Math.Abs(x - i) < allowedDistance || Math.Abs(y - j) < allowedDistance))                        
                        return false;                    
                }
            }

            return true;
        }
    }

    public class Ground : Tile
    {
        public Ground() : this(0, 0) { }
        public Ground(int x, int y) : base(x, y) { }
        public override bool IsPassable() => true;
        public override void PlaceInstance(Tile[][] map, int x, int y, AdjacencyMatrix matrix)
        {
            if (CheckForAllowToPlace(map, x, y, matrix, typeof(Ground)))
                map[x][y] = new Ground(x, y);
        }
    }

    public class Water : Tile
    {
        public Water() : this(0, 0) { }
        public Water(int x, int y) : base(x, y) { }
        public override bool IsPassable() => false;
        public override void PlaceInstance(Tile[][] map, int x, int y, AdjacencyMatrix matrix)
        {
            if (CheckForAllowToPlace(map, x, y, matrix, typeof(Water)))                            
                map[x][y] = new Water(x, y);            
        }
    }

    public class Lava : Tile
    {
        public Lava() : this(0, 0) { }
        public Lava(int x, int y) : base(x, y) { }
        public override bool IsPassable() => false;

        public override void PlaceInstance(Tile[][] map, int x, int y, AdjacencyMatrix matrix)
        {
            if (CheckForAllowToPlace(map, x, y, matrix, typeof(Lava)))
                map[x][y] = new Lava(x, y);
        }
    }

    public class Bridge : Tile
    {
        public Bridge() : this(0, 0) { }
        public Bridge(int x, int y, bool isLast = false) : base(x, y) {}
        public override bool IsPassable() => true;

        public bool IsLast { get; private set; } //концы моста помечаются, чтоб при прокладывании дороги, зайти на мост и покинуть его можно было только на его "концах"

        public override void PlaceInstance(Tile[][] map, int x, int y, AdjacencyMatrix matrix)
        {
            Bridge lastBridge = new Bridge(x, y);
            lastBridge.IsLast = true;

            if (!CheckForAllowToPlace(map, x, y, matrix, typeof(Bridge)))
                return;

            map[x][y] = lastBridge;
            
            Random rnd = new Random(x * y);
            int bridgesCount = rnd.Next(1, 10);
            
            bool xAxis = rnd.Next(0, 2) == 0 ? true : false;
            int direction = rnd.Next(0, 2) == 0 ? 1 : -1;
            
            while (bridgesCount > 0)
            {
                if (xAxis)
                    x += direction;
                else
                    y += direction;

                if (x < 0 || x >= map.Length || y < 0 || y >= map[x].Length)
                    return;

                if (!CheckForAllowToPlace(map, x, y, matrix, typeof(Bridge)))
                    break;

                lastBridge = new Bridge(x, y);
                map[x][y] = lastBridge;

                bridgesCount--;
            }

            lastBridge.IsLast = true;
        }
    }

    public class Road : Tile
    {
        public Road() : this(0, 0) { }
        public Road(int x, int y) : base(x, y) { }
        public override bool IsPassable() => true;
        public override void PlaceInstance(Tile[][] map, int x, int y, AdjacencyMatrix matrix)
        {
            if (CheckForAllowToPlace(map, x, y, matrix, typeof(Road)))
                map[x][y] = new Road(x, y);
        }
    }

    #region A* path
    public class APathFinder
    {
        private class PathNode
        {            
            public Tile Tile { get; set; }
            public int PathLengthFromStart { get; set; }
            public PathNode CameFrom { get; set; }
            public int HeuristicEstimatePathLength { get; set; } // примерное расстояние до цели

            public int EstimateFullPathLength => this.PathLengthFromStart + this.HeuristicEstimatePathLength; // ожидаемое полное расстояние до цели
        }

        public static List<Tile> FindPath(Tile[][] field, Tile start, Tile goal)
        {
            if (start == null || goal == null) return null;

            var closedSet = new List<PathNode>();
            var openSet = new List<PathNode>();
         
            PathNode startNode = new PathNode()
            {
                Tile = start,
                CameFrom = null,
                PathLengthFromStart = 0,
                HeuristicEstimatePathLength = GetHeuristicPathLength(start, goal)
            };
            openSet.Add(startNode);
            while (openSet.Count > 0)
            {         
                var currentNode = openSet.OrderBy(node => node.EstimateFullPathLength).First();
             
                if (currentNode.Tile.X == goal.X && currentNode.Tile.Y == goal.Y)
                    return GetPathForNode(currentNode);
                
                openSet.Remove(currentNode);
                closedSet.Add(currentNode);
                
                foreach (var neighbourNode in GetNeighbours(currentNode, goal, field))
                {                
                    if (closedSet.Count(node => node.Tile.X == neighbourNode.Tile.X && node.Tile.Y == neighbourNode.Tile.Y) > 0)
                        continue;
                    var openNode = openSet.FirstOrDefault(node => node.Tile.X == neighbourNode.Tile.X && node.Tile.Y == neighbourNode.Tile.Y);
                 
                    if (openNode == null)
                        openSet.Add(neighbourNode);
                    else if (openNode.PathLengthFromStart > neighbourNode.PathLengthFromStart)
                    {                 
                        openNode.CameFrom = currentNode;
                        openNode.PathLengthFromStart = neighbourNode.PathLengthFromStart;
                    }
                }
            }            
            return null;
        }

        private static int GetDistanceBetweenNeighbours() => 1;

        private static int GetHeuristicPathLength(Tile from, Tile to)
        {
            if (from == null || to == null)
                return 0;

            return Math.Abs(from.X - to.X) + Math.Abs(from.Y - to.Y);
        }

        private static List<PathNode> GetNeighbours(PathNode pathNode, Tile goal, Tile[][] field)
        {
            var result = new List<PathNode>();
            
            Tuple<int, int>[] neighbourPoints = new Tuple<int,int>[4];
            
            neighbourPoints[0] = new Tuple<int,int>(pathNode.Tile.X + 1, pathNode.Tile.Y);
            neighbourPoints[1] = new Tuple<int,int>(pathNode.Tile.X - 1, pathNode.Tile.Y);
            neighbourPoints[2] = new Tuple<int,int>(pathNode.Tile.X, pathNode.Tile.Y + 1);
            neighbourPoints[3] = new Tuple<int,int>(pathNode.Tile.X, pathNode.Tile.Y - 1);

            foreach (var point in neighbourPoints)
            {                
                if (point.Item1 < 0 || point.Item1 >= field.Length || point.Item2 < 0 || point.Item2 >= field[point.Item1].Length)
                    continue;

                Tile neighbourTile = field[point.Item1][point.Item2];

                // Проверяем, что по клетке можно ходить.
                if (neighbourTile == null || !neighbourTile.IsPassable()) continue;

                //Проверяем, можем ли мы зайти на мост / сойти с моста
                if (pathNode.Tile is Bridge currentBridge && !currentBridge.IsLast && !(neighbourTile is Bridge)) continue;
                if (!(pathNode.Tile is Bridge) && neighbourTile is Bridge neighbourBridge && !neighbourBridge.IsLast) continue;
                
                var neighbourNode = new PathNode()
                {
                    Tile = neighbourTile,
                    CameFrom = pathNode,
                    PathLengthFromStart = pathNode.PathLengthFromStart +
                    GetDistanceBetweenNeighbours(),
                    HeuristicEstimatePathLength = GetHeuristicPathLength(neighbourTile, goal)
                };
                result.Add(neighbourNode);
            }
            return result;
        }

        private static List<Tile> GetPathForNode(PathNode pathNode)
        {
            var result = new List<Tile>();
            var currentNode = pathNode;
            while (currentNode != null)
            {
                result.Add(currentNode.Tile);
                currentNode = currentNode.CameFrom;
            }
            result.Reverse();
            return result;
        }
    }
    #endregion
}
