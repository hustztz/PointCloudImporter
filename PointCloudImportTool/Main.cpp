#include <iostream>
#include <vector>
#include <ctime>
#include <ostream>
#include <istream>
#include <sstream>
#include <Windows.h>

#include <utility/RCStringUtils.h>
#include <utility/RCFilesystem.h>

#include "LASDataSource.h"
#include "DataSourceIndexer.h"

#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Core/Context.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/PointCloud/AgVoxelContainer.h>
#include <Urho3D/PointCloud/AgPointCloudOptions.h>

#pragma warning(disable:4503)

#define LAS_FILE_EXTENSION       /*NOXLATE*/ L".las"
#define LAZ_FILE_EXTENSION       /*NOXLATE*/ L".laz"

using namespace ambergris::RealityComputing::Common;
using namespace ambergris::RealityComputing::Import;
using namespace ambergris::RealityComputing::Utility;
using namespace ambergris::PointCloudEngine;

int importLAS(const std::wstring &filename, const std::wstring &outputPath, Urho3D::Node* pNode, bool createNormals) {
	if (!Filesystem::exists(filename))
	{
		std::wcerr << L"input file " << filename << L" does not exist\n";
		return -1;
	}
	auto fileExtension = Filesystem::extension(filename);
	if (fileExtension != LAS_FILE_EXTENSION && fileExtension != LAZ_FILE_EXTENSION)
	{
		std::wcerr << "The input file format is not supported\n";
		return -1;
	}
	clock_t startTime = clock(); //Start timer
	LASDataSource *dataSource = new LASDataSource(filename);
	if (!dataSource)
	{
		std::wcerr << L"Fail to create las data source";
		return -1;
	}

	bool result = dataSource->open();
	if (!result)
	{
		std::wcerr << L"Fail to open data source \n";
		delete dataSource;
		dataSource = nullptr;
		return -1;
	}

	std::wstring tempDir;
#ifdef _WIN32
	Filesystem::createTempDirectory(tempDir, L"Ambergris\\Urho3D");
#else
	Filesystem::createTempDirectory(tempDir, L"Ambergris/Urho3D");
#endif

	std::wstring errLog;
	DataSourceIndexer<ExtendedOctreeImportPoint> importer(*dataSource, filename);
	importer.setTempFolder(tempDir);
	importer.setOutputFolder(outputPath);
	std::wstring scanName = Filesystem::basename(filename);
	importer.setInput(scanName, filename);
	//importer.setFilterType(NOISE_FILTER_MEDIUM);
	if (createNormals) {
		importer.setCalculateNormalsFromUnstructuredData(true);
	}
	else {
		importer.setCalculateNormalsFromUnstructuredData(false);
	}

	std::wcerr << L"\nProcessing: " << scanName << std::endl;

	std::wcerr << L"Generate temp files\n";
	auto parseCode = importer.parsePoints(errLog);
	if (parseCode != IRCUserDataSource::ErrorCode::OK)
	{
		std::wcerr << L"Fail to parse points \n";
	}

	std::wcerr << L"Close\n";
	importer.postParse(errLog);

	std::wcerr << L"Indexing\n";
	auto success = importer.convertToNativeFormat(errLog);
	if (!success)
	{
		std::wcerr << L"Fail to convert format. \n";
	}
	
	if (!pNode)
		return -1;

	const RCVector3d& translation = importer.getTranslation();
	pNode->SetPosition(Urho3D::Vector3((float)translation.x, (float)translation.y, (float)translation.z));
	const RCVector3d& rotation = importer.getRotation();
	pNode->SetRotation(Urho3D::Quaternion((float)rotation.x, (float)rotation.y, (float)rotation.z));
	const RCVector3d& scale = importer.getScale();
	pNode->SetScale(Urho3D::Vector3((float)scale.x, (float)scale.y, (float)scale.z));

	RCRotationMatrix rotMat(rotation, scale);
	RCTransform transform = RCTransform(rotMat, translation);

	pNode->SetVar(VoxelTreeRunTimeVars::VAR_LIDARDATA, Urho3D::Variant(true));
	const RCVector3d& scannerOrigin = importer.getScannerOrigin();
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_SCANNERORIGIN, Urho3D::Variant(Urho3D::Vector3((float)scannerOrigin.x, (float)scannerOrigin.y, (float)scannerOrigin.z)));
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_TOTALPOINTS, Urho3D::Variant(importer.getTotalAmountOfPoints()));
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_HASRGB, Urho3D::Variant(importer.hasRGB()));
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_HASNORMALS, Urho3D::Variant(importer.hasNormals()));
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_HASINTENSITY, Urho3D::Variant(importer.hasIntensity()));
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_POINTSIZE, Urho3D::Variant(5.0f));
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_MATERIAL, Urho3D::Variant(Urho3D::ResourceRef(Urho3D::Material::GetTypeStatic(), "E:\\Projects\\V8\\Urho3D\\bin\\Data\\Materials\\PointCloud.xml")));

	RCBox svoBounds = importer.getSVOBounds();
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_SVOBOUNDSMIN, Urho3D::Variant(Urho3D::Vector3((float)svoBounds.getMin().x, (float)svoBounds.getMin().y, (float)svoBounds.getMin().z)));
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_SVOBOUNDSMAX, Urho3D::Variant(Urho3D::Vector3((float)svoBounds.getMax().x, (float)svoBounds.getMax().y, (float)svoBounds.getMax().z)));
	RCBox scanBounds = importer.getScanBounds();
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_SCANBOUNDSMIN, Urho3D::Variant(Urho3D::Vector3((float)scanBounds.getMin().x, (float)scanBounds.getMin().y, (float)scanBounds.getMin().z)));
	pNode->SetVar(VoxelTreeRunTimeVars::VAR_SCANBOUNDSMAX, Urho3D::Variant(Urho3D::Vector3((float)scanBounds.getMax().x, (float)scanBounds.getMax().y, (float)scanBounds.getMax().z)));

	std::vector<OctreeLeafSaveData> leafSaveList;
	importer.convertLeafNodeToLeafSaveData(leafSaveList);

	std::uint64_t numLeafs = static_cast<std::uint64_t>(leafSaveList.size());
	//fill in voxel containers
	for (int i = 0; i < int(numLeafs); i++)
	{
		const OctreeLeafSaveData& curLeaf = leafSaveList[i];
		AgVoxelContainer* runTimeLod = pNode->CreateComponent<AgVoxelContainer>();
		//runTimeLod->m_nodeBounds = curLeaf.m_nodeBounds;

		runTimeLod->SetSVOBoundsMin(Urho3D::Vector3((float)curLeaf.m_svoBounds.getMin().x, (float)curLeaf.m_svoBounds.getMin().y, (float)curLeaf.m_svoBounds.getMin().z));
		runTimeLod->SetSVOBoundsMax(Urho3D::Vector3((float)curLeaf.m_svoBounds.getMax().x, (float)curLeaf.m_svoBounds.getMax().y, (float)curLeaf.m_svoBounds.getMax().z));
		runTimeLod->SetAmountOfPoints(curLeaf.m_amountOfPoints);
		runTimeLod->SetMaxLOD((char)curLeaf.m_maxDepth);
		runTimeLod->SetResourceName(Urho3D::String(curLeaf.m_fileName.c_str()));
	}

	clock_t stopTime = clock();
	std::wcerr << L"Time taken " << ((double)(stopTime - startTime) / CLOCKS_PER_SEC) << "s\n";
	return 0;
}

int main(int argc, char* argv[])
{
	const std::vector<std::string> cmdline(argv, argv + argc);


	std::wstring outputPath;
	std::vector<std::wstring> inputScans;

	if (cmdline.size() < 5) {
		return 1;
	}

	if (cmdline[1].compare("--outputDir") == 0)
	{
		outputPath = String::RCStringUtils::string2WString(cmdline[2]);
		if (!Filesystem::isDirectory(outputPath)) {
			std::wcerr << L"output directory " << outputPath << L" does not exist\n";
			return -1;
		}
	}
	else {
		return 1;
	}

	if (cmdline[3].compare("--inputScan") == 0)
	{
		inputScans.push_back(String::RCStringUtils::string2WString(cmdline[4]));
	}
	else if (cmdline[3].compare("--inputDir") == 0)
	{
		//get all the las/laz files in the input directory
		std::wstring directoryName = String::RCStringUtils::string2WString(cmdline[4]);
		if (!Filesystem::isDirectory(directoryName)) {
			std::wcerr << L"input scan directory " << directoryName << L" does not exist\n";
			return -1;
		}

		std::vector<std::wstring> fileNames = Filesystem::listFiles(directoryName);
		for (auto& file : fileNames)
		{
			if (Filesystem::extension(file) == L".las" || Filesystem::extension(file) == L".laz")
				inputScans.push_back(file);
		}
	}

	//optional argument: --normals 0/1
	bool createNormals = false;
	auto it = std::find(cmdline.begin(), cmdline.end(), "--normals");
	if (it != cmdline.end()) {
		auto index = std::distance(cmdline.begin(), it);
		createNormals = std::stoi(cmdline[index + 1]) == 1 ? true : false;
	}

	Urho3D::SharedPtr<Urho3D::Context> context(new Urho3D::Context());

	context->RegisterSubsystem(new Urho3D::FileSystem(context));
	context->RegisterSubsystem(new Urho3D::ResourceCache(context));
	Urho3D::RegisterSceneLibrary(context);
	Urho3D::RegisterGraphicsLibrary(context);

	Urho3D::SharedPtr<Urho3D::Node> rootNode(new Urho3D::Node(context));
	AgPointCloudOptions* pointCloudOptions = rootNode->CreateComponent<AgPointCloudOptions>();

	for (size_t i = 0; i < inputScans.size(); i++)
	{
		Urho3D::Node* pNode = rootNode->CreateChild(Urho3D::String(inputScans[i].c_str()));
		importLAS(inputScans[i], outputPath, pNode, createNormals);
	}

	Urho3D::PODVector<Urho3D::Node*> pointCloudNodes = rootNode->GetChildren(true);
	if (!pointCloudNodes.Empty() && pointCloudOptions)
	{
		const Urho3D::Vector3& offset = pointCloudNodes[0]->GetVar(VoxelTreeRunTimeVars::VAR_SCANNERORIGIN).GetVector3();
		pointCloudOptions->setOffset(offset);
		for (unsigned i = 0; i < pointCloudNodes.Size(); i ++)
		{
			Urho3D::Vector3 translation = pointCloudNodes[i]->GetPosition() - offset;
			pointCloudNodes[i]->SetPosition(translation);
		}
	}

	Urho3D::File file(context);
	Urho3D::String outName = Urho3D::String((outputPath + std::wstring(L"\\sdsd.xml")).c_str());
	if (file.Open(outName, Urho3D::FILE_WRITE))
	{
		rootNode->SaveXML(file);
	}

	return 0;
}