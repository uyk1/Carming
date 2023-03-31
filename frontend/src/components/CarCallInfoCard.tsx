import {Text, View, Image} from 'react-native';
import {useTheme} from 'react-native-paper';
import styled from 'styled-components';

interface CarCallInfoCardProps {
  statusText: string;
  infoText: string;
}

const CarCallInfoCard: React.FC<CarCallInfoCardProps> = ({
  statusText,
  infoText,
}) => {
  return (
    <CardContainerView>
      <CarmingCarImage
        source={require('../assets/images/carming_loading.gif')}
      />
      <StatusText>{statusText}</StatusText>
      <InfoText style={{color: useTheme().colors.primary}}>{infoText}</InfoText>
    </CardContainerView>
  );
};

const CardContainerView = styled(View)`
  justify-content: center;
  align-items: center;
  padding: 20px 0px;
`;

const StatusText = styled(Text)`
  font-size: 12px;
  text-align: center;
  font-weight: bold;
  margin: 20px 0px;
`;

const InfoText = styled(Text)`
  font-size: 16px;
  text-align: center;
  font-weight: bold;
`;

const CarmingCarImage = styled(Image)`
  width: 135px;
  height: 120px;
`;

export default CarCallInfoCard;
